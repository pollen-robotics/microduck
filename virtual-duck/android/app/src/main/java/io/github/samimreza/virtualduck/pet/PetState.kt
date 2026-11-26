package io.github.samimreza.virtualduck.pet

import org.json.JSONObject

enum class Mood { HAPPY, CONTENT, HUNGRY, SLEEPY, LONELY, BORED }

/**
 * What the duck needs and how it feels about it, what the two of you have built up, and the few
 * things about it that are its own: a name, a voice, a size, where on the screen it lives.
 *
 * Needs run 0 (empty) to 1 (satisfied) and drain with time, so a duck left alone wants feeding,
 * company and a game when it is next seen — but never drains past [FLOOR]. A pet that can be
 * neglected into a state that takes an hour to undo is a pet people uninstall, and this one is
 * meant to be a pleasure to come back to: every day it is looked after adds to a streak, and every
 * kindness adds to a friendship level, which in time teaches it to sing.
 */
class PetState(
    var name: String = "Duck",
    /** Which of the voices `tools/render_voices.py` rendered it speaks in: babble, duckling or squeaky. */
    var voice: String = DEFAULT_VOICE,
    /** 0 low, 1 middle, 2 high. */
    var voicePitch: Int = 1,
    var food: Float = 0.8f,
    var energy: Float = 0.9f,
    var joy: Float = 0.7f,
    var love: Float = 0.7f,
    var asleep: Boolean = false,
    /** 0 silent, 1 soft, 2 normal. */
    var soundLevel: Int = 1,
    /** Answers when spoken to — the microphone, used for loudness only. Off until asked for. */
    var talk: Boolean = false,
    /** How tall the duck stands on screen, dp. */
    var sizeDp: Int = 150,
    /** Where its floor is: 1 the bottom of the screen, 0 as high as it fits. */
    var floorLevel: Float = 1f,
    var xp: Int = 0,
    var streak: Int = 0,
    /** The last day it was looked after, as days since 1970 in the phone's time zone. */
    var lastCareDay: Long = 0L,
    var pelletsEaten: Int = 0,
    var timesPetted: Int = 0,
    var kicks: Int = 0,
    var chats: Int = 0,
    var remindedDay: Long = 0L,
    var remindersToday: Int = 0,
    var bornAtMs: Long = 0L,
    var updatedAtMs: Long = 0L,
) {
    val muted: Boolean get() = soundLevel == 0

    val mood: Mood
        get() = when {
            energy < 0.2f -> Mood.SLEEPY
            food < 0.25f -> Mood.HUNGRY
            love < 0.25f -> Mood.LONELY
            joy < 0.25f -> Mood.BORED
            minOf(food, energy, joy, love) > 0.65f -> Mood.HAPPY
            else -> Mood.CONTENT
        }

    val level: Int get() = levelFor(xp)

    /** XP gathered since the current level began, and how much the next one needs. */
    val levelProgress: Pair<Int, Int>
        get() = (xp - threshold(level)) to (threshold(level + 1) - threshold(level))

    val canSing: Boolean get() = level >= SING_LEVEL

    /**
     * Lets [seconds] pass. Awake, everything drains; asleep, energy comes back and the rest drains
     * at half the rate, the way it does for anything asleep. [active] is the duck walking or
     * playing, which tires it faster than standing about; [charging] is a nap on the charger, which
     * restores energy as sleep does.
     */
    fun elapse(seconds: Float, active: Boolean = false, charging: Boolean = false) {
        val hours = seconds / 3600f
        if (asleep) {
            energy += ENERGY_SLEEP * hours
            food -= FOOD * hours / 2
            joy -= JOY * hours / 2
            love -= LOVE * hours / 2
        } else {
            energy -= (if (active) ENERGY_ACTIVE else ENERGY) * hours
            if (charging) energy += ENERGY_SLEEP * hours
            food -= FOOD * hours
            joy -= JOY * hours
            love -= LOVE * hours
        }
        clamp()
    }

    /**
     * Catches up on the time nobody was watching: the app closed, the duck put away, the phone off.
     * A duck that is not on screen is resting, so it is aged as asleep — and only so far back, so a
     * long absence costs no more than [AWAY_CAP_HOURS] would.
     */
    fun catchUp(nowMs: Long) {
        if (updatedAtMs > 0L && nowMs > updatedAtMs) {
            val seconds = ((nowMs - updatedAtMs) / 1000f).coerceAtMost(AWAY_CAP_HOURS * 3600f)
            val wasAsleep = asleep
            asleep = true
            elapse(seconds)
            asleep = wasAsleep
        }
        updatedAtMs = nowMs
    }

    /**
     * Something kind happened on [today]. The first kindness of a day extends the streak if
     * yesterday had one too, and starts it again if not. Returns true when the streak grew.
     */
    fun cared(today: Long): Boolean {
        if (today == lastCareDay) return false
        streak = if (today == lastCareDay + 1) streak + 1 else 1
        lastCareDay = today
        return true
    }

    /** Adds [amount] friendship. Returns true when that was a new level. */
    fun gain(amount: Int): Boolean {
        val before = level
        xp += amount
        return level > before
    }

    fun eat() {
        food += 0.2f
        joy += 0.02f
        pelletsEaten++
        clamp()
    }

    fun petted() {
        love += 0.04f
        joy += 0.01f
        timesPetted++
        clamp()
    }

    fun played(amount: Float = 0.08f) {
        joy += amount
        energy -= amount / 4
        clamp()
    }

    fun talkedTo() {
        love += 0.03f
        joy += 0.02f
        chats++
        clamp()
    }

    private fun clamp() {
        food = food.coerceIn(FLOOR, 1f)
        energy = energy.coerceIn(FLOOR, 1f)
        joy = joy.coerceIn(FLOOR, 1f)
        love = love.coerceIn(FLOOR, 1f)
    }

    fun toJson(): String = JSONObject()
        .put("name", name)
        .put("voice", voice)
        .put("voice_pitch", voicePitch)
        .put("food", food.toDouble())
        .put("energy", energy.toDouble())
        .put("joy", joy.toDouble())
        .put("love", love.toDouble())
        .put("asleep", asleep)
        .put("sound_level", soundLevel)
        .put("talk", talk)
        .put("size_dp", sizeDp)
        .put("floor_level", floorLevel.toDouble())
        .put("xp", xp)
        .put("streak", streak)
        .put("last_care_day", lastCareDay)
        .put("pellets_eaten", pelletsEaten)
        .put("times_petted", timesPetted)
        .put("kicks", kicks)
        .put("chats", chats)
        .put("reminded_day", remindedDay)
        .put("reminders_today", remindersToday)
        .put("born_at_ms", bornAtMs)
        .put("updated_at_ms", updatedAtMs)
        .toString()

    companion object {
        /** Needs never drain below this; see the class comment. */
        const val FLOOR = 0.05f

        // Per hour. Food empties in about ten waking hours, energy in fourteen and back in five of
        // sleep; joy and love need attention a few times a day.
        const val FOOD = 0.09f
        const val ENERGY = 0.065f
        const val ENERGY_ACTIVE = 0.25f
        const val ENERGY_SLEEP = 0.2f
        const val JOY = 0.11f
        const val LOVE = 0.08f

        /**
         * Half a day. Any longer and food, which keeps draining in sleep, reaches the floor — and
         * whoever comes back after a weekend should find a duck asking for breakfast, not one that
         * looks neglected.
         */
        const val AWAY_CAP_HOURS = 12f

        const val SING_LEVEL = 3

        /**
         * XP a level starts at: 0, 50, 150, 300, 500… At a few dozen kind things a day, level 2
         * comes on the first day and each after takes a little longer — something to look forward
         * to for weeks, not something finished in an afternoon.
         */
        fun threshold(level: Int): Int = 25 * level * (level - 1)

        fun levelFor(xp: Int): Int {
            var level = 1
            while (threshold(level + 1) <= xp) level++
            return level
        }

        const val DEFAULT_VOICE = "babble"

        /** A new duck, hatched now. */
        fun hatch(nowMs: Long) = PetState(bornAtMs = nowMs, updatedAtMs = nowMs)

        fun fromJson(text: String): PetState {
            val j = JSONObject(text)
            return PetState(
                name = j.optString("name", "Duck"),
                // A duck saved with one of the old numbered voices, which are gone, speaks in the default.
                voice = j.optString("voice", DEFAULT_VOICE),
                voicePitch = j.optInt("voice_pitch", 1).coerceIn(0, 2),
                food = j.optDouble("food", 0.8).toFloat(),
                energy = j.optDouble("energy", 0.9).toFloat(),
                joy = j.optDouble("joy", 0.7).toFloat(),
                love = j.optDouble("love", 0.7).toFloat(),
                asleep = j.optBoolean("asleep", false),
                // Saved before sound had levels: muted or not.
                soundLevel = if (j.has("sound_level")) j.getInt("sound_level") else if (j.optBoolean("muted")) 0 else 1,
                talk = j.optBoolean("talk", false),
                sizeDp = j.optInt("size_dp", 150),
                floorLevel = j.optDouble("floor_level", 1.0).toFloat(),
                xp = j.optInt("xp", 0),
                streak = j.optInt("streak", 0),
                lastCareDay = j.optLong("last_care_day", 0L),
                pelletsEaten = j.optInt("pellets_eaten", 0),
                timesPetted = j.optInt("times_petted", 0),
                kicks = j.optInt("kicks", 0),
                chats = j.optInt("chats", 0),
                remindedDay = j.optLong("reminded_day", 0L),
                remindersToday = j.optInt("reminders_today", 0),
                bornAtMs = j.optLong("born_at_ms", 0L),
                updatedAtMs = j.optLong("updated_at_ms", 0L),
            )
        }
    }
}
