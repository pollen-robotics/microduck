package io.github.samimreza.virtualduck.pet

import io.github.samimreza.virtualduck.motion.Animator
import io.github.samimreza.virtualduck.motion.MotionLibrary
import io.github.samimreza.virtualduck.motion.Vec2
import io.github.samimreza.virtualduck.motion.wrapAngle
import java.time.LocalDate
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

/** The duck's voice, as the brain needs it. */
interface Voice {
    /** One sound of [tag] — greet, inquire, chirp, coo, peck, alarm, wheee. Quiet when muted. */
    fun say(tag: String)

    /** How open the beak should be right now, 0–1, from whatever is playing. */
    fun mouth(): Float

    /** Making a sound right now — the ears are shut for it, so it never answers itself. */
    val speaking: Boolean

    /** The duck chorale, in this voice. Returns how long it lasts, 0 if there is no song. */
    fun sing(): Float

    /** Stops the song, if it is singing. */
    fun hush()
}

/** A ball on the floor: simulator metres, its centre. */
class Ball(var x: Float, var z: Float, var vx: Float = 0f, var vz: Float = 0f) {
    /** Rolled angle, for drawing it turning. */
    var angle = 0f

    /** Following a kick's recording rather than its own physics. */
    var scripted = false

    val resting: Boolean get() = !scripted && abs(vx) < 0.03f && z <= RADIUS + 0.001f && abs(vz) < 0.05f

    companion object {
        /** The 70 mm ball the kick policies were trained to kick. */
        const val RADIUS = 0.035f
    }
}

enum class Activity { IDLE, TURNING, WALKING, PECKING, SITTING, SLEEPING, DANCING, KICKING, ROLLING, CARRIED, FLYING, FALLING }

/**
 * What the duck does, and why.
 *
 * Its world is a floor: a line along the simulator's X between [left] and [right], seen from the
 * front — the bottom of the screen unless its owner moves it. It walks along it with the recorded
 * walk and turns with the recorded turns, which go in fixed steps, so a turn is planned to stop at
 * the loop wrap nearest its target and the remainder is steered out while walking. Everything it
 * does is one of the policies' own motions, chosen by its needs, its mood and what is done to it;
 * the only motions it has that no policy made are being carried, thrown and jumping.
 *
 * It listens when talked to and answers in duck, purrs and wiggles when stroked, tumbles when the
 * phone is shaken, greets you when you unlock the phone, naps on the charger. Kindness to it builds
 * a friendship level and a daily streak — kept in [state] — and a new level is celebrated.
 *
 * Buttons make no sound of their own: the duck vocalizes when there is a reason to — eating, being
 * stroked, being thrown, greeting, answering — and not as a click.
 *
 * Pure logic, no Android: the service and the tests drive [update] and read what to draw.
 */
class Brain(
    private val motions: MotionLibrary,
    val animator: Animator,
    val state: PetState,
    private val voice: Voice,
    private val random: Random = Random.Default,
    /** Today, as days since 1970 in the phone's time zone — what a streak counts in. */
    private val today: () -> Long = { LocalDate.now().toEpochDay() },
) {
    var left = 0.1f
    var right = 1.0f

    var activity = Activity.IDLE
        private set

    /** Height of the duck's feet above its floor: in somebody's hand, in the air, mid-jump. */
    var lift = 0f
        private set

    val pellets = mutableListOf<Float>()

    /** Bumped whenever a seed lands or is eaten, so a view redraws seeds only when they change. */
    var pelletsVersion = 0
        private set

    var ball: Ball? = null
        private set

    /** A thought over the duck's head — an emoji — and for how long. */
    var bubble: String? = null
        private set
    private var bubbleLeft = 0f

    /** Hearts waiting to float up from a stroke; the view takes them and counts this back down. */
    var hearts = 0

    /** A new friendship level was reached; for the screen to make something of it. */
    var onLevelUp: ((Int) -> Unit)? = null

    private var clock = 0f

    /** The trunk's heading with the stride's sway smoothed out. */
    private var facing = 0f
    private var decideIn = 2f
    private var lookIn = 1f
    private var targetX = 0f
    private var arrive: (() -> Unit)? = null
    private var turnTarget = 0f
    private var turnClip = ""
    private var afterTurn: (() -> Unit)? = null
    private var pending: (() -> Unit)? = null
    private var sitLeft = 0f
    private var danceLeft = 0
    private var singing = false
    private var singLeft = 0f
    private var biteAt = Float.MAX_VALUE
    private var swing = 0f
    private var vx = 0f
    private var vz = 0f
    private var gentle = false
    private var lastCarryX = 0f
    private var kickReleased = true
    private var kickStartX = 0f
    private var kicksInARow = 0
    private var ballRestingFor = 0f
    private var pecksMissed = 0

    /** Which way the current walk set off, +1 or -1 along the floor: it steers and stops by that. */
    private var walkWay = 1f

    /**
     * The walk or turn under way is one the owner asked for — called over, a roll, a hello — and
     * not one it chose itself: a sound starting does not stop it.
     */
    private var ownerPlan = false

    /** Arrivals without a step taken, this frame and in a row: see [goTo]. */
    private var arrivals = 0
    private var stuck = 0

    /** The owner threw a ball or seeds: it goes for them whatever its mood, until they are done. */
    private var wantsBall = false
    private var wantsFood = false
    private var listening = false
    private var listenLeft = 0f
    private val replies = ArrayDeque<String>()
    private val replyGaps = ArrayDeque<Float>()
    private var nextReplyAt = 0f
    private var napping = false
    private var napLater = false
    private var charging = false
    private val pokes = ArrayDeque<Float>()
    private val strokes = ArrayDeque<Float>()
    private val asked = mutableMapOf<String, Float>()
    private val lastSaid = mutableMapOf<String, Float>()
    private val lastRewarded = mutableMapOf<String, Float>()
    private val ballScratch = FloatArray(3)
    private var prevBallX = 0f
    private var prevBallZ = 0f
    private var slideLeft = 0f
    private var slideRate = 0f
    private var slideThen: (() -> Unit)? = null

    private val peckReach = motions.clip("peck").events["reach_m"] ?: 0.08f
    private val peckBite = motions.clip("peck").events["bite_s"] ?: 1.3f
    private val walkStop = motions.clip("walk_stop").net.x
    private val walkCycle = motions.clip("walk").cycle.x

    init {
        animator.onFinished = ::finished
        animator.onWrapped = ::wrapped
        if (state.asleep) {
            activity = Activity.SLEEPING
            animator.play("sit_idle", 0f)
            sitLeft = Float.MAX_VALUE
        }
        facing = animator.facing
    }

    /** How often this moment is worth drawing: smooth while moving, sparing while still. */
    val frameRate: Int
        get() = when {
            activity == Activity.SLEEPING -> 6
            ball?.let { !it.resting } == true || hearts > 0 -> 30
            (activity == Activity.IDLE || activity == Activity.SITTING) && !listening && replies.isEmpty() -> 20
            else -> 30
        }

    // ---------------------------------------------------------------- what people do to it

    /** A tap on the duck at ([x], [z]), simulator metres. */
    fun poke(x: Float, z: Float) {
        pokes.addLast(clock)
        while (pokes.isNotEmpty() && clock - pokes.first() > 3f) pokes.removeFirst()
        if (pokes.size >= 4 && standing()) {
            // Prodded once too often: it startles and goes over backwards. Then it gets up.
            pokes.clear()
            voice.say("alarm")
            fall("fall_backward")
            return
        }
        when (activity) {
            Activity.SLEEPING -> wake()
            Activity.IDLE, Activity.SITTING -> {
                animator.lookAt(x, -0.4f, z)
                lookIn = 2f
                say("greet", cooldown = 4f)
                reward("poke", 1, cooldown = 5f)
            }
            Activity.WALKING, Activity.TURNING -> stopMoving()
            else -> Unit
        }
    }

    /**
     * A stroke across the duck at ([x], [z]). Called once per stroke, not per touch event. A few
     * strokes in a row and it wiggles with pleasure.
     */
    fun stroke(x: Float, z: Float) {
        state.petted()
        hearts++
        say("coo", cooldown = 1.5f)
        animator.lookAt(x, -0.5f, z)
        animator.tiltHead(if (random.nextBoolean()) 0.2f else -0.2f)
        lookIn = 2f
        reward("pet", 2, cooldown = 1f)
        if (activity == Activity.WALKING || activity == Activity.TURNING) stopMoving()
        strokes.addLast(clock)
        while (strokes.isNotEmpty() && clock - strokes.first() > 6f) strokes.removeFirst()
        if (strokes.size >= 5 && activity == Activity.IDLE) {
            strokes.clear()
            hearts += 3
            show("😊", 2f)
            say("chirp", cooldown = 2f)
            startDance(cycles = 1)
        }
    }

    fun grab(x: Float) {
        cancel()
        activity = Activity.CARRIED
        lastCarryX = x
        swing = 0f
        if (!animator.isPlaying("stand")) animator.play("stand", 0.2f)
        say("inquire", cooldown = 3f)
    }

    /** The finger holding the duck is at floor position [x], [height] above the floor. */
    fun carry(x: Float, height: Float, dt: Float) {
        if (activity != Activity.CARRIED) return
        val speed = if (dt > 0f) (x - lastCarryX) / dt else 0f
        lastCarryX = x
        lift = height.coerceAtLeast(0f)
        animator.place(x.coerceIn(left, right), animator.floorY, animator.facing)
        // Held from above, it trails: dragged right, its feet lag left and its top leans right.
        val want = (speed * 0.35f).coerceIn(-0.7f, 0.7f)
        swing += (want - swing) * (1f - exp(-10f * dt))
    }

    /** Let go, moving at ([vx], [vz]) m/s. */
    fun release(vx: Float, vz: Float) {
        if (activity != Activity.CARRIED) return
        this.vx = vx.coerceIn(-4f, 4f)
        this.vz = vz.coerceIn(-4f, 4f)
        gentle = false
        activity = Activity.FLYING
        if (hypot(vx, vz) > 1.5f) voice.say("wheee")
    }

    /** Taken away mid-carry — dropped on the dismiss target. Nothing more happens to it. */
    fun putAway() {
        cancel()
        activity = Activity.IDLE
        lift = 0f
        swing = 0f
        say("coo")
    }

    /**
     * Jumps to floor position [x] on a floor that has just moved: [from] is where its feet now are
     * relative to the new floor — below it, negative, when the floor rose. An arc with its top a
     * little over the higher end, timed so it lands at [x]; and a jump always lands on its feet.
     */
    fun jump(x: Float, from: Float) {
        cancel()
        // Whatever it was doing ends here: its clip must not end mid-air and land it in IDLE.
        if (!animator.isPlaying("stand")) animator.play("stand", 0.15f)
        lift = from
        val peak = maxOf(from, 0f) + JUMP_CLEARANCE
        vz = sqrt(2 * GRAVITY * (peak - from))
        val flight = vz / GRAVITY + sqrt(2 * peak / GRAVITY)
        vx = (x.coerceIn(left, right) - animator.floorX) / flight
        gentle = true
        activity = Activity.FLYING
        say("wheee", cooldown = 2f)
    }

    /**
     * Arrives on screen: dropped in from a little above its floor, onto its feet, with a hello — or,
     * asleep, simply there, still asleep.
     */
    fun dropIn() {
        if (activity == Activity.SLEEPING) return
        cancel()
        if (!animator.isPlaying("stand")) animator.play("stand", 0.15f)
        lift = 0.18f
        vx = 0f
        vz = 0f
        gentle = true
        activity = Activity.FLYING
    }

    /**
     * Walks to floor position [x] — the owner pointed there — and turns to them. Not right up to an
     * edge: the walk's shortest step would carry it past it, half off the screen.
     */
    fun come(x: Float) {
        whenFree {
            ownerPlan = true
            goTo(inside(x, EDGE_MARGIN), closeEnough = 0.05f) { faceViewer() }
        }
    }

    fun feed() {
        ownerWants()
        if (pellets.size >= 5) return
        // Far enough that the duck walks to it, which is the fun of throwing food — but inside the
        // floor it has, which at home is a strip of screen not much wider than a few strides.
        val reach = minOf(0.4f, (right - left) * 0.45f)
        var toward = if (random.nextBoolean()) 1f else -1f
        if (animator.floorX + toward * reach !in left..right) toward = -toward
        val x = inside(animator.floorX + toward * reach * (0.6f + random.nextFloat() * 0.4f), SEED_MARGIN)
        pellets += x
        pelletsVersion++
        wantsFood = true
    }

    fun throwBall() {
        ownerWants()
        val toward = if (animator.floorX < (left + right) / 2) 1f else -1f
        val x = inside(animator.floorX + toward * minOf(0.3f, (right - left) * 0.45f), SEED_MARGIN)
        ball = Ball(x, 0.35f, vx = toward * 0.2f)
        kicksInARow = 0
        wantsBall = true
    }

    fun flickBall(vx: Float) {
        val b = ball ?: return
        if (b.scripted) return
        ownerWants()
        b.vx = vx.coerceIn(-3f, 3f)
        b.vz = 0.6f
        kicksInARow = 0
        wantsBall = true
    }

    /**
     * The owner pressed a button for it: whatever can be cut short is — a walk stops at its next
     * stride, a sitting duck gets up, a sleeping one wakes, a dance ends at its next beat — and it
     * stops listening, since a button is its owner talking to it as plainly as can be.
     */
    private fun ownerWants() {
        stopListening()
        replies.clear()
        when (activity) {
            Activity.SITTING -> sitLeft = 0f
            Activity.SLEEPING -> wake()
            Activity.WALKING, Activity.TURNING -> stopMoving()
            Activity.DANCING -> {
                danceLeft = minOf(danceLeft, 1)
                endSong()
            }
            else -> Unit
        }
    }

    /** Something of the owner's is waiting for it: seeds on the floor, or the ball. */
    private fun ownerBusiness() = (wantsFood && pellets.isNotEmpty()) || (wantsBall && ball != null)

    /** A dance, now or as soon as the duck is free to. */
    fun dance() {
        if (activity == Activity.DANCING) return
        if (activity != Activity.IDLE) show("🎵", 1.5f)
        whenFree { startDance() }
    }

    /** The forward roll — unless it is too tired, which it says. */
    fun trick() {
        if (state.energy < 0.3f) {
            show("😴", 2f)
            return
        }
        if (activity != Activity.IDLE) show("🤸", 1.5f)
        // The roll goes forward almost half a metre: facing you, it tumbles toward you rather than
        // off the side of the screen.
        whenFree {
            ownerPlan = true
            turnThen(VIEWER) { startTrick() }
        }
    }

    /** The duck chorale, danced to — once friendship has got that far. */
    fun sing() {
        if (!state.canSing) {
            show("🔒", 2f)
            return
        }
        if (singing) return
        show("🎶", 1.5f)
        whenFree { startSinging() }
    }

    fun toggleSleep() {
        when (activity) {
            Activity.SLEEPING -> wake()
            Activity.SITTING -> sleep()
            else -> whenFree { sleep() }
        }
    }

    /**
     * Somebody started talking: it turns its head to them and tilts it to listen, and stops unless it
     * is on its way to something its owner gave it or asked of it. A sound that never turns out to be speech — a
     * door, a cough, the video in the app underneath — ends with [stopListening], or a timeout.
     */
    fun listen() {
        if (activity == Activity.SLEEPING || !awakeAndAround()) return
        listening = true
        listenLeft = LISTEN_MAX
        replies.clear()
        if ((activity == Activity.WALKING || activity == Activity.TURNING) && !ownerBusiness() && !ownerPlan) stopMoving()
        animator.lookAt(animator.floorX, -1f, 0.25f)
        animator.tiltHead(0.22f)
        lookIn = 4f
    }

    /** What it was listening to stopped without having been speech: it goes back to what it was doing. */
    fun stopListening() {
        if (!listening) return
        listening = false
        animator.tiltHead(0f)
        lookIn = 1f
    }

    /**
     * They stopped: it answers in duck — as many sounds as they said syllables, up to six, in their
     * rhythm. A short call gets a questioning sound back; a long speech a run of chirps ending in one.
     */
    fun heard(syllables: Int, gaps: List<Float>) {
        listening = false
        if (activity == Activity.SLEEPING) {
            show("💤", 1.5f)
            return
        }
        if (!awakeAndAround()) return
        val count = syllables.coerceIn(1, MAX_REPLY)
        replies.clear()
        replyGaps.clear()
        val words = listOf("chirp", "peck", "coo", "chirp")
        for (i in 0 until count) {
            replies += when {
                count == 1 -> "inquire"
                i == count - 1 -> if (random.nextBoolean()) "inquire" else "greet"
                else -> words[(i + random.nextInt(words.size)) % words.size]
            }
            replyGaps += (gaps.getOrNull(i) ?: 0.3f).coerceIn(0.12f, 0.6f)
        }
        nextReplyAt = clock + 0.25f
        state.talkedTo()
        reward("talk", 3, cooldown = 2f)
        if (activity == Activity.IDLE && !ownerBusiness()) faceViewer()
    }

    /** The phone was shaken: it loses its footing, and gets back up. */
    fun shaken(sideways: Float) {
        if (!awakeAndAround() || activity == Activity.SITTING) return
        voice.say("alarm")
        val f = cos(animator.facing)
        fall(
            when {
                abs(f) > 0.7f -> if (sideways * f > 0f) "fall_backward" else "fall_forward"
                else -> if (sideways > 0f) "fall_right" else "fall_left"
            },
        )
    }

    /** The phone was just unlocked: it turns to say hello. */
    fun greetUser() {
        if (activity == Activity.SLEEPING || !awakeAndAround()) return
        if (clock - (asked["unlock"] ?: -1000f) < 90f) return
        asked["unlock"] = clock
        whenFree {
            ownerPlan = true
            turnThen(VIEWER) {
                say("greet")
                show("👋", 2.5f)
                reward("unlock", 1, cooldown = 60f)
            }
        }
    }

    /**
     * On the charger: a tired duck takes a nap there — once it is free, and has eaten the seeds and
     * played with the ball its owner gave it — and wakes when it is unplugged.
     */
    fun charging(on: Boolean) {
        charging = on
        napLater = on && state.energy < 0.85f && activity != Activity.SLEEPING
        if (!on && napping && activity == Activity.SLEEPING) wake()
    }

    /**
     * Catches up with the duck on the other screen, which shares [state] and may have gone to sleep
     * or woken up while this one was out of sight.
     */
    fun sync() {
        if (state.asleep && activity != Activity.SLEEPING) {
            if (activity == Activity.SITTING || standing()) sleep() else whenFree { if (state.asleep) sleep() }
        } else if (!state.asleep && activity == Activity.SLEEPING) {
            wake()
        }
    }

    /**
     * Out of sight — its screen hidden or the phone's screen off: a song stops rather than playing on
     * from a duck nobody can see, and an answer it had not finished is dropped.
     */
    fun silence() {
        if (singing) {
            endSong()
            danceLeft = minOf(danceLeft, 1)
        }
        replies.clear()
        stopListening()
    }

    /**
     * The floor was rescaled — the duck drawn bigger or smaller, which the screen does by changing
     * how many pixels a metre is. What lies on it moves with it, so a seed stays where it was on the
     * glass rather than off the end of a floor that just got shorter.
     */
    fun rescale(factor: Float) {
        for (i in pellets.indices) pellets[i] *= factor
        pelletsVersion++
        ball?.let { it.x *= factor }
        targetX *= factor
        slideLeft *= factor
        lastCarryX *= factor
        // A kick's recording follows the duck, not the floor: one under way lets go of the ball, which
        // drops where it is rather than being flung by the jump.
        if (!kickReleased) {
            kickReleased = true
            ball?.let {
                it.scripted = false
                it.vx = 0f
                it.vz = 0f
            }
        }
    }

    /** What it is doing and what it has, for the debug bridge and the tests. */
    fun status(): Map<String, Any?> = mapOf(
        "activity" to activity.name.lowercase(),
        "floor_x" to animator.floorX,
        "floor_y" to animator.floorY,
        "left" to left,
        "right" to right,
        "lift" to lift,
        "pellets" to pellets.size,
        "wants_food" to wantsFood,
        "ball" to (ball != null),
        "ball_x" to ball?.x,
        "ball_resting" to ball?.resting,
        "wants_ball" to wantsBall,
        "kicks_in_a_row" to kicksInARow,
        "listening" to listening,
        "replies" to replies.size,
        "pending" to (pending != null),
        "bubble" to bubble,
        "asleep" to state.asleep,
        "voice" to "${state.voice}-${state.voicePitch}",
        "food" to state.food,
        "energy" to state.energy,
        "joy" to state.joy,
        "love" to state.love,
        "level" to state.level,
        "xp" to state.xp,
        "pellets_eaten" to state.pelletsEaten,
        "kicks" to state.kicks,
        "chats" to state.chats,
    )

    // ---------------------------------------------------------------- every frame

    fun update(dt: Float) {
        clock += dt
        arrivals = 0
        facing += wrapAngle(animator.facing - facing) * (1f - exp(-dt / 0.35f))
        state.elapse(
            dt,
            active = activity != Activity.IDLE && activity != Activity.SITTING && activity != Activity.SLEEPING,
            charging = charging,
        )
        bubbleLeft -= dt
        if (bubbleLeft <= 0f) bubble = if (activity == Activity.SLEEPING) "💤" else null

        when (activity) {
            Activity.WALKING -> steerToward()
            Activity.PECKING -> if (animator.time >= biteAt) bite()
            Activity.KICKING -> followKick(dt)
            Activity.FLYING -> fly(dt)
            Activity.SLEEPING -> if (state.energy >= 0.97f) wake()
            Activity.SITTING -> {
                sitLeft -= dt
                if (sitLeft <= 0f && animator.isPlaying("sit_idle")) standUp()
            }
            Activity.DANCING -> if (singLeft > 0f) singLeft -= dt
            else -> Unit
        }
        if (activity != Activity.WALKING) animator.steerRate = 0f
        if (slideLeft != 0f) {
            val step = minOf(abs(slideLeft), slideRate * dt) * sign(slideLeft)
            animator.nudge(step, 0f)
            slideLeft -= step
            if (abs(slideLeft) < 1e-5f) finishSlide()
        }
        // Rolls, falls and turns leave it nearer the camera or further from it than its floor line.
        // It drifts back at a pace nobody sees, to stay on the line its seeds and ball lie on.
        val y = animator.floorY
        if (abs(y) > 0.002f && activity != Activity.ROLLING && activity != Activity.FALLING) {
            animator.nudge(0f, -sign(y) * minOf(abs(y), DEPTH_RETURN * dt))
        }
        if (listening) {
            listenLeft -= dt
            if (listenLeft <= 0f) stopListening()
        }
        rollBall(dt)
        tidyUp(dt)
        answer()

        animator.tilt(cos(swing / 2), 0f, sin(swing / 2), 0f)
        if (activity != Activity.CARRIED && activity != Activity.FLYING) swing *= exp(-6f * dt)
        animator.mouth = voice.mouth()
        animator.advance(dt)

        if (activity == Activity.IDLE || activity == Activity.SITTING) {
            lookIn -= dt
            if (lookIn <= 0f && !listening && replies.isEmpty()) lookAround()
        }
        if (activity == Activity.IDLE) {
            // Whatever the owner asked for is done, or was given up.
            ownerPlan = false
            val asked = pending
            val b = ball
            when {
                asked != null -> {
                    pending = null
                    asked()
                }
                // What its owner gave it comes before anything it was minding, listening included.
                wantsFood && pellets.isNotEmpty() -> eatNearest()
                wantsBall && b != null && kicksInARow < MAX_KICKS ->
                    if (b.resting) chase(b) else animator.lookAt(b.x, 0f, b.z)
                napLater && charging -> nap()
                listening || replies.isNotEmpty() -> Unit
                else -> {
                    decideIn -= dt
                    if (decideIn <= 0f) decide()
                }
            }
        }
        // A plan that keeps arriving where it already is, frame after frame — which a sound floor
        // never makes, but a floor is not always sound — is given up rather than stood in forever.
        if (arrivals > MAX_ARRIVALS) {
            if (++stuck > 90) {
                stuck = 0
                wantsFood = false
                wantsBall = false
                show("🤷", 2f)
            }
        } else if (activity != Activity.IDLE) {
            stuck = 0
        }
    }

    /**
     * Lets go of what is finished: the seeds are all eaten, the ball has had its kicks. A ball left
     * lying untouched for [BALL_FORGOTTEN] seconds is put away, so it does not sit on the screen.
     */
    private fun tidyUp(dt: Float) {
        if (pellets.isEmpty()) wantsFood = false
        // Seeds stay on the floor the duck can reach, however its floor changes.
        if (right - left > 2 * SEED_MARGIN) {
            for (i in pellets.indices) {
                val p = pellets[i]
                val inside = p.coerceIn(left + SEED_MARGIN, right - SEED_MARGIN)
                if (inside != p) {
                    pellets[i] = inside
                    pelletsVersion++
                }
            }
        }
        val b = ball
        if (b == null) {
            wantsBall = false
            return
        }
        if (kicksInARow >= MAX_KICKS && wantsBall) {
            wantsBall = false
            show("😮‍💨", 2f)
        }
        ballRestingFor = if (b.resting && !wantsBall && activity != Activity.KICKING) ballRestingFor + dt else 0f
        if (ballRestingFor > BALL_FORGOTTEN) {
            ball = null
            ballRestingFor = 0f
        }
    }

    /** Says the next sound of an answer once the last has finished and its gap has passed. */
    private fun answer() {
        if (replies.isEmpty() || voice.speaking || clock < nextReplyAt) return
        voice.say(replies.removeFirst())
        // A small bob of the head with each "word".
        animator.tiltHead(if (replies.size % 2 == 0) 0.18f else -0.12f)
        nextReplyAt = clock + replyGaps.removeFirstOrNull().let { it ?: 0.3f }
        if (replies.isEmpty()) {
            animator.tiltHead(0f)
            lookIn = 2f
        }
    }

    // ---------------------------------------------------------------- choosing

    private fun decide() {
        decideIn = 4f + random.nextFloat() * 5f
        val b = ball
        when {
            // Its floor can shrink under it — a bigger size, a rotated screen: walk back onto it.
            animator.floorX !in left..right -> goTo(inside(animator.floorX, 0.05f), closeEnough = 0.02f) { }
            pellets.isNotEmpty() && state.food < 0.97f -> eatNearest()
            b != null && b.resting && state.joy < 0.97f && state.energy > 0.25f && kicksInARow < 5 -> chase(b)
            state.mood == Mood.SLEEPY -> sleep()
            state.mood == Mood.HUNGRY && ask("🌾") -> Unit
            state.mood == Mood.LONELY && ask("❤️") -> Unit
            state.mood == Mood.BORED && ask("⚽") -> Unit
            else -> {
                val roll = random.nextFloat()
                when {
                    roll < 0.35f -> wander()
                    roll < 0.5f -> faceViewer()
                    roll < 0.6f -> peck()
                    roll < 0.72f || state.energy < 0.45f -> sit(6f + random.nextFloat() * 10f)
                    roll < 0.82f && state.joy > 0.6f -> startDance()
                    roll < 0.84f && state.joy > 0.8f && state.energy > 0.6f -> startTrick()
                    roll < 0.9f -> say("chirp", cooldown = 20f)
                    else -> Unit
                }
            }
        }
    }

    /** Turns to the viewer and asks for something, at most once every 45 s per thing. */
    private fun ask(what: String): Boolean {
        if (clock - (asked[what] ?: -1000f) < 45f) return false
        asked[what] = clock
        turnThen(VIEWER) {
            say("inquire")
            show(what, 3f)
        }
        return true
    }

    private fun lookAround() {
        lookIn = 2f + random.nextFloat() * 3f
        animator.tiltHead(0f)
        val roll = random.nextFloat()
        when {
            roll < 0.5f -> animator.lookAt(animator.floorX, -1f, 0.2f)
            roll < 0.8f -> animator.lookAt(animator.floorX + (random.nextFloat() - 0.5f) * 1.2f, -0.3f, random.nextFloat() * 0.3f)
            else -> animator.lookAway()
        }
    }

    /**
     * Does [action] now if the duck is free, or as soon as it is: a walk stops at its next stride, a
     * sitting duck stands up, a sleeping one wakes, anything else is finished first. A button that
     * did nothing because the duck happened to be busy is a button that looks broken.
     */
    private fun whenFree(action: () -> Unit) {
        when (activity) {
            Activity.IDLE -> action()
            Activity.WALKING, Activity.TURNING -> {
                pending = action
                stopMoving()
            }
            Activity.SITTING -> {
                pending = action
                if (!animator.isPlaying("sit_up")) standUp()
            }
            Activity.SLEEPING -> {
                pending = action
                wake()
            }
            Activity.DANCING -> {
                pending = action
                danceLeft = minOf(danceLeft, 1)
                endSong()
            }
            else -> pending = action
        }
    }

    /**
     * Friendship for something kind, at most once per [cooldown] seconds per [kind] so stroking
     * frantically is not a way to farm it. The first kindness of a day extends the streak.
     */
    private fun reward(kind: String, xp: Int, cooldown: Float = 0f) {
        if (clock - (lastRewarded[kind] ?: -1000f) < cooldown) return
        lastRewarded[kind] = clock
        if (state.cared(today()) && state.streak in STREAK_MILESTONES) show("🔥", 3f)
        if (state.gain(xp)) {
            show("🎉", 3f)
            hearts += 4
            onLevelUp?.invoke(state.level)
            if (activity != Activity.DANCING) whenFree { startDance(cycles = 2) }
        }
    }

    // ---------------------------------------------------------------- moving

    private fun wander() {
        val span = right - left
        var x = left + random.nextFloat() * span
        if (abs(x - animator.floorX) < 0.2f) x = (animator.floorX + if (x > animator.floorX) 0.25f else -0.25f)
        goTo(inside(x, EDGE_MARGIN)) { if (random.nextFloat() < 0.5f) faceViewer() }
    }

    private fun faceViewer() = turnThen(VIEWER) { }

    /**
     * Walks to floor position [x] and then does [then]. Closer than a walk's own start would carry it
     * — about 14 cm — counts as there: the walk cannot take a smaller step than that.
     *
     * Already there, [then] runs at once — and [then] is often a plan that comes straight back here.
     * Planned soundly it has moved on by then; but so that a plan that has not, against a wall or on a
     * floor that just shrank, is never a recursion that ends with the stack, arriving goes only
     * [MAX_ARRIVALS] deep in a frame and the rest waits for the next.
     */
    private fun goTo(x: Float, closeEnough: Float = MIN_WALK, then: () -> Unit) {
        targetX = x.coerceIn(left, right)
        arrive = then
        val distance = targetX - animator.floorX
        if (abs(distance) < closeEnough) {
            if (++arrivals > MAX_ARRIVALS) {
                arrive = null
                activity = Activity.IDLE
                return
            }
            arrived()
            return
        }
        turnThen(if (distance > 0f) 0f else PI.toFloat()) { startWalk() }
    }

    private fun arrived() {
        val then = arrive
        arrive = null
        activity = Activity.IDLE
        then?.invoke()
    }

    /**
     * Slides the duck [dx] along the floor over [seconds], then does [then]. For the last few
     * centimetres the walk cannot do — its shortest step is about 14 cm — done while the duck is
     * bending or settling, at a few centimetres a second, which nobody sees as sliding.
     */
    private fun slide(dx: Float, seconds: Float, then: (() -> Unit)? = null) {
        slideLeft = dx
        slideRate = abs(dx) / seconds
        slideThen = then
        if (dx == 0f) finishSlide()
    }

    private fun finishSlide() {
        slideLeft = 0f
        val then = slideThen
        slideThen = null
        then?.invoke()
    }

    private fun turnThen(heading: Float, then: () -> Unit) {
        val error = wrapAngle(heading - animator.facing)
        if (abs(error) < TURN_TOLERANCE) {
            then()
            return
        }
        activity = Activity.TURNING
        turnTarget = heading
        afterTurn = then
        turnClip = if (error > 0f) "turn_left" else "turn_right"
        animator.play("${turnClip}_start", 0.2f)
    }

    /** True once the turn loop has come close enough that its stop will finish the job. */
    private fun turnDone(): Boolean {
        val remaining = wrapAngle(turnTarget - animator.facing)
        val way = if (turnClip == "turn_left") 1f else -1f
        val step = abs(motions.clip(turnClip).cycle.yaw)
        val stop = motions.clip("${turnClip}_stop").net.yaw * way
        return remaining * way <= step / 2 + stop
    }

    private fun startWalk() {
        activity = Activity.WALKING
        walkWay = if (targetX >= animator.floorX) 1f else -1f
        animator.play("walk_start", 0.2f)
    }

    /** True once the next wrap would carry the walk past its target, stop and all. */
    private fun walkDone(): Boolean {
        val ahead = (targetX - animator.floorX) * walkWay
        return ahead <= walkStop + walkCycle / 2
    }

    /**
     * Keeps the walk on the floor line, heading for [x]. The aim is the line's own direction plus a
     * capped correction for sideways drift — not the target point, which close up turns a few
     * millimetres of drift into a hard swerve — and it is compared with a smoothed heading, because
     * the trunk sways a good ten degrees either way inside every stride. The line's direction is the
     * one the walk set off in: a walk that has passed its target stops, it does not swing round.
     */
    private fun steerToward() {
        val forward = walkWay > 0f
        val correction = (animator.floorY * DRIFT_GAIN).coerceIn(-0.25f, 0.25f)
        val want = if (forward) -correction else PI.toFloat() + correction
        animator.steerRate = (wrapAngle(want - facing) * 1.5f).coerceIn(-0.5f, 0.5f)
    }

    private fun stopMoving() {
        arrive = null
        afterTurn = null
        ownerPlan = false
        when (activity) {
            Activity.WALKING -> if (animator.isPlaying("walk")) animator.queue("walk_stop") else targetX = animator.floorX
            Activity.TURNING -> if (animator.isPlaying(turnClip)) animator.queue("${turnClip}_stop") else turnTarget = animator.facing
            else -> Unit
        }
    }

    // ---------------------------------------------------------------- doing

    /**
     * Goes for the nearest seed: to the spot a peck's reach short of it, facing it. Within [SLIDE_MAX]
     * of that spot it turns to the seed if it has to, plans again from where the turn left it, then
     * pecks and slides the rest during the lean. Further, it walks there — the shortest walk's
     * overshoot is inside the slide. Next to a wall, where that spot is off the floor, it goes round
     * to the seed's other side.
     */
    private fun eatNearest() {
        val seed = pellets.minBy { abs(it - animator.floorX) }
        val way = approach(seed, peckReach)
        val spot = seed - way * peckReach
        val heading = if (way > 0f) 0f else PI.toFloat()
        when {
            abs(spot - animator.floorX) > SLIDE_MAX -> goTo(spot, closeEnough = SLIDE_MAX) { eatNearest() }
            !facing(heading) -> turnThen(heading) { if (pellets.isNotEmpty()) eatNearest() }
            else -> peck(spot - animator.floorX)
        }
    }

    /**
     * Which way to face something at [x] to reach it from [reach] away: from the duck's side, unless
     * that spot is off the floor — then from the other.
     */
    private fun approach(x: Float, reach: Float): Float {
        val dx = x - animator.floorX
        val way = if (abs(dx) > 0.01f) sign(dx) else cos(animator.facing).sign
        return if (x - way * reach in left..right || x + way * reach !in left..right) way else -way
    }

    private fun facing(heading: Float) = abs(wrapAngle(heading - animator.facing)) < TURN_TOLERANCE

    private fun peck(shift: Float = 0f) {
        animator.lookAway()
        activity = Activity.PECKING
        biteAt = peckBite
        animator.play("peck", 0.2f)
        // Done by the time the beak is down.
        slide(shift, peckBite * 0.8f)
    }

    /**
     * The bottom of the peck: a seed within reach of the beak is eaten. A duck that has missed twice
     * running reaches a little further, so no seed is ever pecked at for good.
     */
    private fun bite() {
        biteAt = Float.MAX_VALUE
        val beak = animator.ground.apply(Vec2(peckReach, 0f)).x
        val near = pellets.minByOrNull { abs(it - beak) }
        if (near != null && abs(near - beak) < if (pecksMissed >= 2) 0.15f else 0.07f) {
            pecksMissed = 0
            pellets.remove(near)
            pelletsVersion++
            if (state.food >= 0.99f) show("😋", 1.5f)
            state.eat()
            voice.say("peck")
            reward("eat", 3)
        } else if (near != null) {
            pecksMissed++
        }
    }

    private fun sit(seconds: Float) {
        cancel()
        activity = Activity.SITTING
        sitLeft = seconds
        animator.play("sit_down", 0.25f)
    }

    private fun standUp() {
        sitLeft = Float.MAX_VALUE
        animator.play("sit_up", 0.2f)
    }

    private fun sleep() {
        // Getting up is SITTING too: a duck part-way up sits back down.
        val rising = activity == Activity.SITTING && animator.isPlaying("sit_up")
        val seated = activity == Activity.SITTING && !rising
        if (!seated && !rising && !standing()) return
        state.asleep = true
        if (!seated) {
            cancel()
            animator.play("sit_down", 0.25f)
        }
        activity = Activity.SLEEPING
        sitLeft = Float.MAX_VALUE
        droop()
        show("💤", 2f)
    }

    /** The nap on the charger. */
    private fun nap() {
        napLater = false
        napping = true
        show("🔌", 2.5f)
        sleep()
    }

    private fun wake() {
        state.asleep = false
        napping = false
        activity = Activity.SITTING
        animator.lookAway()
        animator.tiltHead(0f)
        standUp()
    }

    /** Head down, the way it sleeps: looking at the floor a little in front. */
    private fun droop() {
        val ahead = cos(animator.facing)
        animator.lookAt(animator.floorX + ahead * 0.25f, animator.floorY + sin(animator.facing) * 0.25f, -0.4f)
    }

    private fun startDance(cycles: Int = 3) {
        cancel()
        activity = Activity.DANCING
        danceLeft = cycles
        animator.play("dance", 0.3f)
    }

    private fun startTrick() {
        cancel()
        activity = Activity.ROLLING
        animator.play("roulade", 0.2f)
    }

    /** The song stops, however it ends: at its end, cut short, or put out of sight. */
    private fun endSong() {
        if (singing) voice.hush()
        singing = false
        singLeft = 0f
    }

    private fun startSinging() {
        val seconds = voice.sing()
        if (seconds <= 0f) return
        startDance()
        singing = true
        singLeft = seconds
        animator.lookAt(animator.floorX, -1f, 0.35f)
        reward("sing", 3, cooldown = 30f)
    }

    /**
     * Goes for the ball the way [eatNearest] goes for a seed, to where the kick expects it. Next to a
     * wall it comes at it from the open side, and kicks it toward the wall to bounce back.
     */
    private fun chase(b: Ball) {
        val way = approach(b.x, KICK_DISTANCE)
        val spot = b.x - way * KICK_DISTANCE
        val heading = if (way > 0f) 0f else PI.toFloat()
        when {
            abs(spot - animator.floorX) > SLIDE_MAX -> goTo(spot, closeEnough = SLIDE_MAX) { chaseAgain() }
            !facing(heading) -> turnThen(heading) { chaseAgain() }
            else -> kick()
        }
    }

    /** Plans for the ball again from here — if it is still there and still. */
    private fun chaseAgain() {
        ball?.let { if (it.resting) chase(it) }
    }

    /** Lines up — the kick was recorded with the ball exactly here — then kicks. */
    private fun kick() {
        val b = ball ?: return
        val way = cos(animator.facing).sign
        val offset = b.x - (animator.floorX + way * KICK_DISTANCE)
        activity = Activity.KICKING
        kickReleased = true
        // Slow enough that lining up reads as a shuffle, not a skate.
        slide(offset, maxOf(0.4f, abs(offset) / 0.12f)) { startKick() }
    }

    private fun startKick() {
        val b = ball ?: return
        if (!b.resting) {
            idle()
            return
        }
        kickReleased = false
        kicksInARow++
        b.scripted = true
        val clip = if (random.nextBoolean()) "kick_left" else "kick_right"
        animator.play(clip, 0.15f)
        animator.ballInto(ballScratch)
        kickStartX = ballScratch[0]
        prevBallX = ballScratch[0]
        prevBallZ = ballScratch[2]
    }

    /** The ball follows the recording until the foot has sent it on its way, then rolls on its own. */
    private fun followKick(dt: Float) {
        val b = ball ?: return
        if (kickReleased || !animator.ballInto(ballScratch)) return
        b.x = ballScratch[0]
        b.z = ballScratch[2]
        val moved = abs(b.x - kickStartX)
        if (moved > 0.04f && dt > 0f) {
            kickReleased = true
            b.scripted = false
            b.vx = (b.x - prevBallX) / dt
            b.vz = (b.z - prevBallZ) / dt
            state.played(0.1f)
            state.kicks++
            reward("kick", 4)
        }
        prevBallX = b.x
        prevBallZ = b.z
    }

    private fun rollBall(dt: Float) {
        val b = ball ?: return
        if (b.scripted) return
        // On the floor and not bouncing, it is in contact: no gravity to bounce it on its own weight
        // whenever a frame runs long.
        if (b.z > Ball.RADIUS || b.vz > 0f) {
            b.vz -= GRAVITY * dt
            b.z += b.vz * dt
        }
        if (b.z <= Ball.RADIUS) {
            b.z = Ball.RADIUS
            b.vz = if (b.vz < -0.4f) -b.vz * 0.5f else 0f
            // Rolling resistance, so a kicked ball comes to rest in a second or two.
            b.vx -= sign(b.vx) * minOf(abs(b.vx), 0.35f * dt)
        }
        b.x += b.vx * dt
        // Its walls are the duck's: a ball that rests where the duck cannot stand cannot be kicked.
        if (b.x < left || b.x > right) {
            b.x = b.x.coerceIn(left, right)
            b.vx = -b.vx * 0.7f
        }
        b.angle -= b.vx / Ball.RADIUS * dt
    }

    private fun fly(dt: Float) {
        vz -= GRAVITY * dt
        var x = animator.floorX + vx * dt
        if (x < left || x > right) {
            x = x.coerceIn(left, right)
            vx = -vx * 0.4f
        }
        lift += vz * dt
        if (!gentle) swing += (-vx * 2.5f).coerceIn(-6f, 6f) * dt
        animator.place(x, animator.floorY, animator.facing)
        // Below the floor on the way up — a jump to a higher one — is not landing.
        if (lift <= 0f && vz < 0f) land()
    }

    private fun land() {
        lift = 0f
        val impact = hypot(vx, vz)
        val tipped = abs(wrapAngle(swing)) > 0.7f
        swing = 0f
        if (!gentle && (impact > 1.1f || tipped)) {
            voice.say("alarm")
            // Which way it goes over is which way it was moving, seen from the duck.
            val f = animator.facing
            val forward = cos(f) * vx
            val leftward = -sin(f) * vx
            fall(
                when {
                    abs(vx) < 0.2f -> if (random.nextBoolean()) "fall_forward" else "fall_backward"
                    abs(forward) >= abs(leftward) -> if (forward > 0f) "fall_forward" else "fall_backward"
                    else -> if (leftward > 0f) "fall_left" else "fall_right"
                },
            )
        } else {
            activity = Activity.IDLE
            animator.play("stand", 0.15f)
            if (gentle) {
                say("greet", cooldown = 3f)
                faceViewer()
            } else if (state.joy > 0.5f) {
                say("chirp", cooldown = 3f)
            }
        }
        gentle = false
        vx = 0f
        vz = 0f
    }

    private fun fall(clip: String) {
        cancel()
        activity = Activity.FALLING
        animator.play(clip, 0.1f)
    }

    // ---------------------------------------------------------------- the clips' callbacks

    private fun finished(name: String) {
        // In the air or in a hand, a clip is only a pose to hold: what it would have led to is moot.
        if (activity == Activity.FLYING || activity == Activity.CARRIED) {
            animator.play("stand", if (name == "stand") 0.5f else 0.2f)
            return
        }
        when {
            name == "stand" -> animator.play("stand", 0.5f)
            name == "sit_idle" -> animator.play("sit_idle", 0.5f)
            name == "sit_down" -> animator.play("sit_idle", 0.1f)
            name == "sit_up" -> when (activity) {
                Activity.SITTING -> idle()
                Activity.SLEEPING -> animator.play("sit_down", 0.25f)
                else -> Unit
            }
            name.startsWith("turn_") && name.endsWith("_start") ->
                animator.play(if (turnDone()) "${turnClip}_stop" else turnClip, 0.08f)
            name.startsWith("turn_") && name.endsWith("_stop") -> {
                activity = Activity.IDLE
                animator.play("stand", 0.15f)
                val then = afterTurn
                afterTurn = null
                then?.invoke()
            }
            name == "walk_start" -> animator.play(if (walkDone()) "walk_stop" else "walk", 0.08f)
            name == "walk_stop" -> {
                animator.steerRate = 0f
                animator.play("stand", 0.15f)
                arrived()
            }
            name == "peck" -> {
                idle()
                if (pellets.isNotEmpty() && state.food < 0.97f) decideIn = 0.3f
            }
            name.startsWith("kick_") -> {
                ball?.scripted = false
                kickReleased = true
                idle()
                decideIn = 1.5f
            }
            name == "roulade" || name.startsWith("fall_") -> {
                state.played(0.05f)
                idle()
                // A roll or a fall can carry it off the edge of the floor; walk back on.
                if (animator.floorX < left || animator.floorX > right) {
                    goTo((left + right) / 2) { }
                }
            }
        }
    }

    private fun wrapped(name: String) {
        if (activity == Activity.FLYING || activity == Activity.CARRIED) return
        when (name) {
            "walk" -> if (walkDone()) animator.queue("walk_stop")
            "turn_left", "turn_right" -> if (turnDone()) animator.queue("${name}_stop")
            "dance" -> {
                state.played(0.05f)
                // A song is danced to its end; otherwise, the cycles asked for.
                val done = if (singing) singLeft <= 0f else --danceLeft <= 0
                if (done) {
                    endSong()
                    reward("dance", 2, cooldown = 10f)
                    idle()
                    animator.play("stand", 0.4f)
                }
            }
        }
    }

    // ---------------------------------------------------------------- small things

    private fun idle() {
        activity = Activity.IDLE
        if (!animator.isPlaying("stand")) animator.play("stand", 0.2f)
        decideIn = 2f + random.nextFloat() * 3f
    }

    private fun cancel() {
        if (activity == Activity.SLEEPING) {
            state.asleep = false
            napping = false
        }
        // A kick cut short before the ball left the foot: the ball is its own again.
        ball?.scripted = false
        kickReleased = true
        arrive = null
        afterTurn = null
        pending = null
        slideLeft = 0f
        slideThen = null
        biteAt = Float.MAX_VALUE
        endSong()
        ownerPlan = false
        animator.steerRate = 0f
        animator.lookAway()
        animator.tiltHead(0f)
        lift = 0f
    }

    /** [x] on the floor, at least [margin] from either end of it when the floor is wide enough. */
    private fun inside(x: Float, margin: Float): Float =
        if (right - left > 2 * margin) x.coerceIn(left + margin, right - margin) else (left + right) / 2

    private fun standing() = activity == Activity.IDLE || activity == Activity.WALKING || activity == Activity.TURNING

    /** Not in somebody's hand, in the air, or on its back. */
    private fun awakeAndAround() =
        activity != Activity.CARRIED && activity != Activity.FLYING && activity != Activity.FALLING

    private fun show(what: String, seconds: Float) {
        bubble = what
        bubbleLeft = seconds
    }

    private fun say(tag: String, cooldown: Float = 0f) {
        if (clock - (lastSaid[tag] ?: -1000f) < cooldown) return
        lastSaid[tag] = clock
        voice.say(tag)
    }

    companion object {
        /** Facing the viewer: the camera looks along +Y, so the duck faces -Y. */
        val VIEWER = (-PI / 2).toFloat()
        const val TURN_TOLERANCE = 0.45f
        const val MIN_WALK = 0.14f

        /**
         * The furthest [slide] goes: what a peck's second-long bend or a kick's shuffle can hide, and
         * more than the shortest walk overshoots by, so a walk never ends where it can do neither.
         */
        const val SLIDE_MAX = 0.10f

        /** How far a plan may arrive where it already is in one frame before the rest waits: see [goTo]. */
        const val MAX_ARRIVALS = 3

        /** Kept clear of the floor's ends by walks it chooses or is called on: the walk overshoots. */
        const val EDGE_MARGIN = 0.08f
        const val SEED_MARGIN = 0.03f

        /** Metres a second it drifts back to its floor line after a roll or a fall. */
        const val DEPTH_RETURN = 0.06f

        /** The longest it listens to a sound that does not end: past the longest speech it answers. */
        const val LISTEN_MAX = 10f

        /** Radians of correction per metre the walk has drifted off the floor line. */
        const val DRIFT_GAIN = 4f

        /** Where the kick policies were trained to find the ball: 9 cm in front of the trunk. */
        const val KICK_DISTANCE = 0.09f

        /** Lighter than Earth's: a thrown duck should hang in the air long enough to enjoy. */
        const val GRAVITY = 6f

        /** How far a jump to a new floor clears the higher of its two ends. */
        const val JUMP_CLEARANCE = 0.1f

        /** The most sounds an answer has, however long it was spoken to. */
        const val MAX_REPLY = 6

        /** Kicks it gives a thrown ball before it has had enough — until the ball is thrown again. */
        const val MAX_KICKS = 8
        const val BALL_FORGOTTEN = 45f

        val STREAK_MILESTONES = setOf(2, 3, 7, 14, 30, 50, 100, 365)
    }
}
