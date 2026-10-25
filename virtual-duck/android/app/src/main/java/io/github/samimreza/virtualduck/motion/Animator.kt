package io.github.samimreza.virtualduck.motion

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sin
import kotlin.math.sqrt

/** Where the animator puts a pose: the model on screen, or a fake in a test. */
fun interface Body {
    /**
     * [frame] is trunk x y z, trunk w x y z — in the simulator's world frame — then the joint angles
     * in [MotionLibrary.joints] order. Called once per rendered frame with the same array, so a body
     * must copy what it keeps.
     */
    fun setPose(frame: FloatArray, mouth: Float)
}

/**
 * Plays recorded motions on the duck and carries it across the floor while they play.
 *
 * Every clip is stored relative to where the duck stood when it began, so the animator keeps one
 * thing of its own: the ground frame — x, y and heading — the current clip started on. A clip
 * starts from wherever the duck is standing now; a loop adds its cycle's displacement on every
 * wrap, so the walk loop walks rather than marching on the spot. Anything that does not join
 * exactly — a walk cut short by a fall, a sit that ends a step back from the stand that follows —
 * is crossfaded.
 *
 * On top of the recording go what the policies never commanded: the head turning toward a point,
 * the beak, a slow steer of the ground frame that lets a straight walk curve toward a target, and a
 * tilt for when the duck dangles from somebody's finger.
 *
 * [advance] runs every frame, so it allocates nothing: every pose is a float array owned here.
 */
class Animator(private val library: MotionLibrary, private val body: Body) {
    var onFinished: ((String) -> Unit)? = null
    var onWrapped: ((String) -> Unit)? = null

    var clip: Clip = library.clip("stand")
        private set
    var time = 0f
        private set

    /** Radians per second the ground frame turns about the trunk: the brain's steering wheel. */
    var steerRate = 0f
    var mouth = 0f

    private val size = library.frameSize
    private val local = FloatArray(size)
    private val world = FloatArray(size)
    private val from = FloatArray(size)
    private val out = FloatArray(size)
    private val scratch = FloatArray(3)
    private var hasPose = false
    private var gx = 0f
    private var gy = 0f
    private var gyaw = 0f
    private var next: String? = null
    private var fadeTime = 0f
    private var fadeLeft = 0f
    private var tw = 1f
    private var tx = 0f
    private var ty = 0f
    private var tz = 0f
    private var looking = false
    private var lookX = 0f
    private var lookY = 0f
    private var lookZ = 0f
    private var lookYaw = 0f
    private var lookPitch = 0f
    private var lookWeight = 0f
    private val headYaw = TRUNK + library.joints.indexOf("head_yaw")
    private val headPitch = TRUNK + library.joints.indexOf("head_pitch")
    private val neckPitch = TRUNK + library.joints.indexOf("neck_pitch")
    private val headRoll = TRUNK + library.joints.indexOf("head_roll")
    private var rollWant = 0f
    private var roll = 0f

    /**
     * Tilts the head sideways by [angle] radians — the listening pose, the quizzical look. Eased
     * there and back; kept inside the ±0.31 rad the policies ever drove head_roll to.
     */
    fun tiltHead(angle: Float) {
        rollWant = angle.coerceIn(-0.3f, 0.3f)
    }

    init {
        play("stand", 0f)
    }

    val queued: String? get() = next
    val ground: Ground get() = Ground(gx, gy, gyaw)
    val floorX: Float get() = world[0]
    val floorY: Float get() = world[1]
    val height: Float get() = world[2]
    val facing: Float get() = headingOf(world[3], world[4], world[5], world[6])

    fun isPlaying(name: String) = clip.name == name

    /** Starts [name] from where the duck stands now, blending into it over [fade] seconds. */
    fun play(name: String, fade: Float = 0.2f) {
        clip = library.clip(name)
        time = 0f
        next = null
        if (hasPose) {
            gx = world[0]
            gy = world[1]
            gyaw = facing
            world.copyInto(from)
        }
        fadeTime = fade
        fadeLeft = if (hasPose) fade else 0f
        apply(0f)
    }

    /**
     * Starts [name] when the current clip ends — or, for a loop, at its next wrap, so that a stop
     * clip begins on the loop's own first pose.
     */
    fun queue(name: String) {
        next = name
    }

    /**
     * Moves the duck so it stands at ([x], [y]) facing [yaw], mid-clip and unblended — for when it
     * is carried, or lands somewhere other than where its motion would have taken it.
     */
    fun place(x: Float, y: Float, yaw: Float) {
        val moved = Ground(x, y, yaw) * Ground(world[0], world[1], facing).inverse() * Ground(gx, gy, gyaw)
        gx = moved.x
        gy = moved.y
        gyaw = moved.yaw
        fadeLeft = 0f
        apply(0f)
    }

    /** Turns the head toward ([x], [y], [z]) in the simulator's frame. */
    fun lookAt(x: Float, y: Float, z: Float) {
        looking = true
        lookX = x
        lookY = y
        lookZ = z
    }

    /** Hands the head back to the motion. */
    fun lookAway() {
        looking = false
    }

    /** Shifts the ground frame by ([dx], [dy]): a centimetre over a frame lines a kick up unseen. */
    fun nudge(dx: Float, dy: Float) {
        gx += dx
        gy += dy
    }

    /** Swings the whole duck about its trunk — dangling, mid-air — by this rotation (w x y z). */
    fun tilt(w: Float, x: Float, y: Float, z: Float) {
        tw = w
        tx = x
        ty = y
        tz = z
    }

    /** Where the current clip has the ball, into [out] as room x y z — kicks only. */
    fun ballInto(out: FloatArray): Boolean {
        if (!clip.ballInto(time, out)) return false
        val c = cos(gyaw)
        val s = sin(gyaw)
        val lx = out[0]
        val ly = out[1]
        out[0] = gx + c * lx - s * ly
        out[1] = gy + s * lx + c * ly
        return true
    }

    fun advance(dt: Float) {
        time += dt
        if (steerRate != 0f) steer(steerRate * dt)
        if (clip.loop) {
            val period = clip.duration
            while (time >= period) {
                time -= period
                val c = cos(gyaw)
                val s = sin(gyaw)
                val d = clip.cycle
                gx += c * d.x - s * d.y
                gy += s * d.x + c * d.y
                gyaw += d.yaw
                onWrapped?.invoke(clip.name)
                if (next != null) {
                    chain()
                    return
                }
            }
        } else if (time >= clip.duration) {
            val ended = clip.name
            if (next != null) chain() else time = clip.duration
            onFinished?.invoke(ended)
            // A listener that started something else has already applied it.
            if (clip.name != ended || time == 0f) return
        }
        fadeLeft = (fadeLeft - dt).coerceAtLeast(0f)
        apply(dt)
    }

    /** Turns the ground frame by [angle] about the trunk's point on the floor. */
    private fun steer(angle: Float) {
        val c = cos(angle)
        val s = sin(angle)
        val px = world[0]
        val py = world[1]
        val dx = gx - px
        val dy = gy - py
        gx = px + c * dx - s * dy
        gy = py + s * dx + c * dy
        gyaw += angle
    }

    private fun chain() {
        val name = next!!
        next = null
        // The recorded joins are exact; the fade only covers the odd millimetre.
        play(name, 0.08f)
    }

    private fun apply(dt: Float) {
        clip.sampleInto(time, local)
        // Local to room: rotate about Z by the ground's heading, then translate.
        val c = cos(gyaw)
        val s = sin(gyaw)
        world[0] = gx + c * local[0] - s * local[1]
        world[1] = gy + s * local[0] + c * local[1]
        world[2] = local[2]
        val hw = cos(gyaw / 2)
        val hz = sin(gyaw / 2)
        world[3] = hw * local[3] - hz * local[6]
        world[4] = hw * local[4] - hz * local[5]
        world[5] = hw * local[5] + hz * local[4]
        world[6] = hw * local[6] + hz * local[3]
        local.copyInto(world, TRUNK, TRUNK, size)
        if (fadeLeft > 0f && fadeTime > 0f) {
            val t = 1f - fadeLeft / fadeTime
            blend(from, 0, world, 0, t * t * (3 - 2 * t), world, size)
        }
        hasPose = true

        world.copyInto(out)
        // The tilt, applied about the trunk: q' = tilt ⊗ q.
        out[3] = tw * world[3] - tx * world[4] - ty * world[5] - tz * world[6]
        out[4] = tw * world[4] + tx * world[3] + ty * world[6] - tz * world[5]
        out[5] = tw * world[5] - tx * world[6] + ty * world[3] + tz * world[4]
        out[6] = tw * world[6] + tx * world[5] - ty * world[4] + tz * world[3]
        normalizeQuat(out, 3)
        overlayLook(dt)
        body.setPose(out, mouth)
    }

    /**
     * Adds a turn of the head toward the look target. Yaw goes to head_yaw; pitch is shared between
     * neck and head the way a duck tips its neck to look up, not just its beak.
     */
    private fun overlayLook(dt: Float) {
        var wantYaw = 0f
        var wantPitch = 0f
        if (looking) {
            // The head sits about here on the trunk; the target, seen from it, in the trunk's frame.
            rotate(world, 0.03f, 0f, 0.12f, inverse = false)
            val hx = world[0] + scratch[0]
            val hy = world[1] + scratch[1]
            val hz = world[2] + scratch[2]
            rotate(world, lookX - hx, lookY - hy, lookZ - hz, inverse = true)
            wantYaw = atan2(scratch[1], scratch[0]).coerceIn(-LOOK_YAW_MAX, LOOK_YAW_MAX)
            wantPitch = atan2(scratch[2], sqrt(scratch[0] * scratch[0] + scratch[1] * scratch[1]))
                .coerceIn(-LOOK_PITCH_MAX, LOOK_PITCH_MAX)
        }
        val follow = if (dt > 0f) 1f - exp(-8f * dt) else 1f
        roll += (rollWant - roll) * (if (dt > 0f) 1f - exp(-6f * dt) else 1f)
        out[headRoll] += roll
        lookYaw += (wantYaw - lookYaw) * follow
        lookPitch += (wantPitch - lookPitch) * follow
        lookWeight += ((if (looking) 1f else 0f) - lookWeight) * (if (dt > 0f) 1f - exp(-4f * dt) else 1f)
        if (lookWeight < 0.001f) return
        out[headYaw] += lookYaw * lookWeight * HEAD_YAW_SIGN
        out[headPitch] += lookPitch * 0.6f * lookWeight * HEAD_PITCH_SIGN
        out[neckPitch] += lookPitch * 0.4f * lookWeight * NECK_PITCH_SIGN
    }

    /** Rotates (x, y, z) by the trunk quaternion in [q] — or its inverse — into [scratch]. */
    private fun rotate(q: FloatArray, x: Float, y: Float, z: Float, inverse: Boolean) {
        val w = q[3]
        val sign = if (inverse) -1f else 1f
        val qx = q[4] * sign
        val qy = q[5] * sign
        val qz = q[6] * sign
        // v + 2w(q×v) + 2q×(q×v)
        val cx = qy * z - qz * y
        val cy = qz * x - qx * z
        val cz = qx * y - qy * x
        scratch[0] = x + 2f * (w * cx + qy * cz - qz * cy)
        scratch[1] = y + 2f * (w * cy + qz * cx - qx * cz)
        scratch[2] = z + 2f * (w * cz + qx * cy - qy * cx)
    }

    companion object {
        /**
         * How far the head turns toward something, radians either way — inside the ranges the
         * policies were trained with (head_yaw ±1.4, pitches ±1.1), with margin.
         */
        const val LOOK_YAW_MAX = 1.1f
        const val LOOK_PITCH_MAX = 0.6f

        /**
         * Which way each joint turns the beak, measured in MuJoCo from the standing pose: +head_yaw
         * swings it left, +neck_pitch tips it up, +head_pitch tips it down — the two pitches oppose,
         * which is how the standing pose (both +0.35) keeps the beak level. AnimatorTest holds these.
         */
        const val HEAD_YAW_SIGN = 1f
        const val HEAD_PITCH_SIGN = -1f
        const val NECK_PITCH_SIGN = 1f
    }
}
