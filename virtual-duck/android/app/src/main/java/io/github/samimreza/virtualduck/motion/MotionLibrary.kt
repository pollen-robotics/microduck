package io.github.samimreza.virtualduck.motion

import org.json.JSONObject
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * A frame is the simulator's qpos for the duck: trunk x y z, trunk quaternion w x y z, then one
 * angle per joint in [MotionLibrary.joints] order — relative to where the duck stood when its clip
 * began. Frames live in one flat array shared by every clip; a clip is an offset into it.
 */
const val TRUNK = 7

/** The heading of a quaternion's X axis about Z — which way a duck with that trunk faces. */
fun headingOf(w: Float, x: Float, y: Float, z: Float): Float =
    atan2(2f * (x * y + w * z), 1f - 2f * (y * y + z * z))

/** One recorded motion: a view into [MotionLibrary]'s data. */
class Clip internal constructor(
    val name: String,
    val loop: Boolean,
    val frameCount: Int,
    private val offset: Int,
    /** Loops: the frame one period on — what the last frame interpolates toward, and how far a cycle goes. */
    private val endOffset: Int,
    private val ballOffset: Int,
    private val data: FloatArray,
    val frameSize: Int,
    val rateHz: Float,
    /** Moments the recorder measured, by name — a peck's `bite_s` and `reach_m`. */
    val events: Map<String, Float> = emptyMap(),
) {
    /** Seconds. A loop's is its period; a one-shot's ends on its last frame. */
    val duration: Float = (if (loop) frameCount else frameCount - 1) / rateHz

    /** How far one cycle of a loop carries the duck; nowhere, for a one-shot. */
    val cycle: Ground = if (loop && endOffset >= 0) groundAt(endOffset) else Ground.ORIGIN

    /** How far the whole of a one-shot carries the duck. */
    val net: Ground = groundAt(offset + (frameCount - 1) * frameSize)

    val hasBall: Boolean get() = ballOffset >= 0

    /** Writes the pose at [t] seconds into [out]. Allocates nothing; runs every frame. */
    fun sampleInto(t: Float, out: FloatArray) {
        var position = t * rateHz
        position = if (loop) {
            ((position % frameCount) + frameCount) % frameCount
        } else {
            position.coerceIn(0f, (frameCount - 1).toFloat())
        }
        val i = position.toInt().coerceAtMost(frameCount - 1)
        val a = offset + i * frameSize
        val b = when {
            i + 1 < frameCount -> a + frameSize
            loop && endOffset >= 0 -> endOffset
            else -> a
        }
        blend(data, a, data, b, position - i, out, frameSize)
    }

    /** The ball's position at [t] into [out] (x y z, clip frame); false if this clip has no ball. */
    fun ballInto(t: Float, out: FloatArray): Boolean {
        if (ballOffset < 0) return false
        val position = (t * rateHz).coerceIn(0f, (frameCount - 1).toFloat())
        val i = position.toInt()
        val j = (i + 1).coerceAtMost(frameCount - 1)
        val f = position - i
        for (k in 0 until 3) {
            val a = data[ballOffset + i * 3 + k]
            out[k] = a + (data[ballOffset + j * 3 + k] - a) * f
        }
        return true
    }

    private fun groundAt(at: Int) = Ground(
        data[at], data[at + 1], headingOf(data[at + 3], data[at + 4], data[at + 5], data[at + 6]),
    )
}

/**
 * Blends two frames: positions and joints linearly, the trunk's quaternion by normalised lerp —
 * indistinguishable from slerp across one 20 ms step, and no trigonometry.
 */
fun blend(a: FloatArray, ai: Int, b: FloatArray, bi: Int, t: Float, out: FloatArray, size: Int) {
    for (k in 0 until size) out[k] = a[ai + k] + (b[bi + k] - a[ai + k]) * t
    // Same hemisphere, or the blend takes the long way round.
    val dot = a[ai + 3] * b[bi + 3] + a[ai + 4] * b[bi + 4] + a[ai + 5] * b[bi + 5] + a[ai + 6] * b[bi + 6]
    if (dot < 0f) for (k in 3 until TRUNK) out[k] = a[ai + k] + (-b[bi + k] - a[ai + k]) * t
    normalizeQuat(out, 3)
}

fun normalizeQuat(q: FloatArray, at: Int) {
    val n = sqrt(q[at] * q[at] + q[at + 1] * q[at + 1] + q[at + 2] * q[at + 2] + q[at + 3] * q[at + 3])
    if (n > 0f) for (k in at until at + 4) q[k] /= n
}

/** The motions `tools/record_motions.py` recorded from the real policies. */
class MotionLibrary(val rateHz: Float, val joints: List<String>, val frameSize: Int, val clips: Map<String, Clip>) {
    fun clip(name: String): Clip = clips[name] ?: error("no clip called $name")

    companion object {
        /** [index] is motions.json; [bin] is motions.bin, little-endian float32. */
        fun load(index: String, bin: ByteBuffer): MotionLibrary {
            val data = FloatArray(bin.remaining() / 4)
            bin.order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(data)
            val root = JSONObject(index)
            val rate = root.getJSONObject("source").getDouble("rate_hz").toFloat()
            val joints = root.getJSONArray("joints").let { a -> List(a.length()) { a.getString(it) } }
            val frameSize = root.getInt("frame_size")
            val clips = mutableMapOf<String, Clip>()
            val entries = root.getJSONObject("clips")
            for (name in entries.keys()) {
                val e = entries.getJSONObject(name)
                val events = e.optJSONObject("events")?.let { o ->
                    o.keys().asSequence().associateWith { o.getDouble(it).toFloat() }
                } ?: emptyMap()
                clips[name] = Clip(
                    name, e.getBoolean("loop"), e.getInt("frames"), e.getInt("offset"),
                    e.getInt("end"), e.getInt("ball"), data, frameSize, rate, events,
                )
            }
            return MotionLibrary(rate, joints, frameSize, clips)
        }
    }
}
