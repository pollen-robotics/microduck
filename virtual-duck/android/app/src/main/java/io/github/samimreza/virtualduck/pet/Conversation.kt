package io.github.samimreza.virtualduck.pet

/**
 * Hears someone talk to the duck, from loudness alone — no speech recognition, nothing recorded,
 * nothing leaves the phone. Fed one level a frame (dBFS, every [frameSeconds]), it tracks the
 * room's background noise, notices speech rising out of it, and when the speaker stops it reports
 * how long they spoke, how many syllables it sounded like, and the gaps between them — which is
 * all a duck needs to answer in duck: as many sounds as you said syllables, in your rhythm.
 *
 * Long unbroken sound — a television, music — is not somebody talking to a duck, so anything past
 * [MAX_SECONDS] is dropped rather than answered, and its level becomes the room's background.
 *
 * Every [onStart] is followed by exactly one [onEnd] or [onCancel]: whoever turned to listen is
 * always told when there is nothing more to listen to.
 */
class Conversation(private val frameSeconds: Float = 0.02f) {
    class Utterance(val seconds: Float, val syllables: Int, val gaps: List<Float>)

    var onStart: (() -> Unit)? = null
    var onEnd: ((Utterance) -> Unit)? = null

    /** What started was not somebody talking after all: too short, too long, or cut off. */
    var onCancel: (() -> Unit)? = null

    /** The room's background level, dBFS. Falls quickly to quiet, rises slowly to noise. */
    var noise = -60f
        private set

    var speaking = false
        private set

    private var loudFrames = 0
    private var quietFrames = 0
    private var ignoring = false
    private var frames = 0
    private var warm = 0
    private var hangover = 0
    private var levels = 0f
    private val peaks = mutableListOf<Int>()
    private var risen = false
    private var valley = 0f
    private var crest = -120f

    /**
     * One frame's level. [deaf] while the duck itself is making a sound — it must not answer its
     * own voice — and for [HANGOVER] after, while the room still rings with it; that also abandons
     * anything half heard.
     */
    fun frame(db: Float, deaf: Boolean = false) {
        if (deaf || hangover > 0) {
            if (deaf) hangover = (HANGOVER / frameSeconds).toInt() else hangover--
            cancel()
            loudFrames = 0
            return
        }
        if (warm < WARMUP_FRAMES) {
            // Just switched on: learn the room before deciding anything is louder than it.
            noise = if (warm == 0) db else noise + (db - noise) * 0.15f
            noise = noise.coerceIn(-90f, -25f)
            warm++
            return
        }
        if (!speaking) {
            noise = if (db < noise) noise + (db - noise) * 0.2f else noise + 0.01f
            noise = noise.coerceIn(-90f, -25f)
        }
        val start = maxOf(noise + START_DB, ABSOLUTE_DB)
        val stop = maxOf(noise + STOP_DB, ABSOLUTE_DB - 6f)
        if (ignoring) {
            // After a sound too long to be talk: deaf until the room is quiet again.
            quietFrames = if (db < stop) quietFrames + 1 else 0
            if (quietFrames * frameSeconds >= END_SILENCE) {
                ignoring = false
                quietFrames = 0
            }
            return
        }
        if (!speaking) {
            loudFrames = if (db > start) loudFrames + 1 else 0
            if (loudFrames >= START_FRAMES) {
                speaking = true
                frames = loudFrames
                levels = db * frames
                quietFrames = 0
                peaks.clear()
                // Speech is only sure a few frames into its first syllable: that one counts now.
                risen = true
                crest = db
                peaks += frames
                valley = db
                onStart?.invoke()
            }
            return
        }
        frames++
        levels += db
        syllables(db, frames)
        quietFrames = if (db < stop) quietFrames + 1 else 0
        val seconds = frames * frameSeconds
        if (seconds > MAX_SECONDS) {
            // Nobody talks to a duck this long: it is the room now — a television, music.
            noise = (levels / frames).coerceIn(-90f, -25f)
            cancel()
            ignoring = true
            return
        }
        if (quietFrames * frameSeconds >= END_SILENCE) {
            val spoke = (frames - quietFrames) * frameSeconds
            val heard = peaks.toList()
            if (spoke >= MIN_SECONDS) {
                reset()
                val gaps = heard.zipWithNext { a, b -> (b - a) * frameSeconds }
                onEnd?.invoke(Utterance(spoke, heard.size.coerceAtLeast(1), gaps))
            } else {
                cancel()
            }
        }
    }

    /**
     * Starts afresh where the ears are switched on again: anything half heard is abandoned, and the
     * room's level is learnt anew rather than carried over from wherever they were last on.
     */
    fun restart() {
        cancel()
        ignoring = false
        loudFrames = 0
        quietFrames = 0
        hangover = 0
        warm = 0
    }

    /** Abandons anything half heard, telling [onCancel] if something was. */
    fun cancel() {
        if (!speaking) return
        reset()
        onCancel?.invoke()
    }

    /**
     * A syllable is a rise of [SYLLABLE_DB] out of the last dip, counted at its top: hysteresis on
     * the level, so the wobble inside one vowel is not counted twice.
     */
    private fun syllables(db: Float, frame: Int) {
        if (!risen) {
            valley = minOf(valley, db)
            if (db > valley + SYLLABLE_DB) {
                risen = true
                crest = db
                peaks += frame
            }
        } else {
            if (db > crest) {
                crest = db
                peaks[peaks.size - 1] = frame
            }
            if (db < crest - SYLLABLE_DB) {
                risen = false
                valley = db
            }
        }
    }

    private fun reset() {
        speaking = false
        loudFrames = 0
        quietFrames = 0
        frames = 0
        levels = 0f
        peaks.clear()
    }

    companion object {
        /** Speech starts this far over the room's noise, and ends when it falls back under [STOP_DB]. */
        const val START_DB = 15f
        const val STOP_DB = 8f

        /** However quiet the room, nothing quieter than this is anybody talking to a duck. */
        const val ABSOLUTE_DB = -50f
        const val START_FRAMES = 4
        const val END_SILENCE = 0.45f
        const val MIN_SECONDS = 0.25f
        const val MAX_SECONDS = 8f
        const val SYLLABLE_DB = 6f

        /** Still deaf this long after the duck's own sound has stopped: its echo, and the speaker's tail. */
        const val HANGOVER = 0.4f

        /** Frames spent learning the room's level when the ears are switched on: about a second. */
        const val WARMUP_FRAMES = 50
    }
}
