package io.github.samimreza.virtualduck.pet

import android.content.Context
import android.media.AudioAttributes
import android.media.SoundPool
import android.os.SystemClock
import android.util.Log
import org.json.JSONObject

/**
 * The duck's voice on a phone: one of the banks `tools/render_voices.py` rendered — a style in a
 * register — played through a SoundPool on the media stream: the volume people turn up for an app's
 * sounds, and not muted by a phone left on vibrate. The in-app sound switch is the way to silence it.
 *
 * SoundPool decodes in the background, so a sound asked for before its bank has loaded — the first
 * seconds after start, or right after a voice is changed — waits for it rather than being dropped.
 *
 * The beak follows the loudness envelope rendered beside each sound rather than an analysis of what
 * is playing, which costs nothing per frame and keeps the beak moving when the sound is muted.
 */
class SoundVoice(private val context: Context, private val state: PetState, private val index: JSONObject) : Voice {
    private class Sound(val tag: String, val id: Int, val duration: Float, val envelope: FloatArray) {
        var ready = false
    }

    private val pool = SoundPool.Builder()
        .setMaxStreams(2)
        .setAudioAttributes(
            AudioAttributes.Builder()
                .setUsage(AudioAttributes.USAGE_GAME)
                .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
                .build(),
        )
        .build()
    private val sounds = mutableMapOf<String, MutableList<Sound>>()
    private val byId = mutableMapOf<Int, Sound>()
    private val last = mutableMapOf<String, Sound>()
    private val envelopeHz = index.getInt("envelope_hz").toFloat()
    private var playing: Sound? = null
    private var startedMs = 0L
    private var songStream = 0
    private var loaded = ""
    private var waitingFor: String? = null
    private var waitingSince = 0L

    init {
        // Called on the main thread, which is where the pool was made.
        pool.setOnLoadCompleteListener { _, id, status ->
            val sound = byId[id] ?: return@setOnLoadCompleteListener
            sound.ready = status == 0
            if (!sound.ready) Log.w(TAG, "could not load ${sound.tag} (status $status)")
            if (sound.ready && waitingFor == sound.tag && SystemClock.uptimeMillis() - waitingSince < WAIT_MS) {
                waitingFor = null
                start(sound)
            }
        }
        reload()
    }

    /**
     * Loads the bank for the duck's voice, if it is not the one already loaded: the default style in
     * the same register if its own is missing, and nothing — a silent duck, not a crash — if the
     * index has neither.
     */
    fun reload() {
        val voices = index.optJSONObject("voices") ?: JSONObject()
        val register = REGISTERS[state.voicePitch.coerceIn(0, REGISTERS.size - 1)]
        val key = listOf("${state.voice}-$register", "${PetState.DEFAULT_VOICE}-$register").firstOrNull { voices.has(it) } ?: ""
        if (loaded == key) return
        for (sound in byId.values) pool.unload(sound.id)
        sounds.clear()
        byId.clear()
        last.clear()
        loaded = key
        if (key.isEmpty()) {
            Log.w(TAG, "could not find a voice for ${state.voice} among ${voices.length()}")
            return
        }
        val voice = voices.getJSONObject(key)
        for (tag in voice.keys()) {
            val list = voice.getJSONArray(tag)
            for (i in 0 until list.length()) {
                val entry = list.getJSONObject(i)
                val levels = entry.getJSONArray("envelope")
                val path = "duck/voices/$key/$tag/${entry.getString("file")}"
                // One sound missing from the package costs that sound, not the app.
                val fd = runCatching { context.assets.openFd(path) }.getOrElse {
                    Log.w(TAG, "could not open $path", it)
                    continue
                }
                fd.use { file ->
                    val sound = Sound(
                        tag,
                        pool.load(file, 1),
                        entry.getDouble("duration").toFloat(),
                        FloatArray(levels.length()) { levels.getDouble(it).toFloat() },
                    )
                    sounds.getOrPut(tag) { mutableListOf() } += sound
                    byId[sound.id] = sound
                }
            }
        }
    }

    override val speaking: Boolean
        get() {
            val sound = playing ?: return false
            return mouthTime() < sound.duration
        }

    /** Sings the duck chorale in this voice. Returns how long it lasts, or 0 if it cannot. */
    override fun sing(): Float {
        val song = sounds["song"]?.firstOrNull { it.ready } ?: return 0f
        songStream = start(song)
        return song.duration
    }

    override fun hush() {
        if (songStream != 0) pool.stop(songStream)
        songStream = 0
        if (playing?.tag == "song") playing = null
    }

    override fun say(tag: String) {
        val ready = sounds[tag]?.filter { it.ready }.orEmpty()
        if (ready.isEmpty()) {
            // Still decoding: play the first of this kind to finish, if it does so soon.
            waitingFor = tag
            waitingSince = SystemClock.uptimeMillis()
            return
        }
        // Never the same variant twice running, so it does not sound like a recording.
        start(ready.filter { it !== last[tag] }.randomOrNull() ?: ready.first())
    }

    /** Plays [sound]; returns its stream, 0 if it is silent. */
    private fun start(sound: Sound): Int {
        last[sound.tag] = sound
        playing = sound
        startedMs = SystemClock.uptimeMillis()
        val volume = VOLUMES[state.soundLevel.coerceIn(0, VOLUMES.size - 1)]
        return if (volume > 0f) pool.play(sound.id, volume, volume, 1, 0, 1f) else 0
    }

    private fun mouthTime() = (SystemClock.uptimeMillis() - startedMs) / 1000f

    override fun mouth(): Float {
        val sound = playing ?: return 0f
        val t = mouthTime()
        if (t >= sound.duration) {
            playing = null
            return 0f
        }
        val i = (t * envelopeHz).toInt().coerceIn(0, sound.envelope.size - 1)
        return sound.envelope[i]
    }

    fun release() = pool.release()

    companion object {
        private const val TAG = "SoundVoice"

        /**
         * Off, soft, normal. Soft is the default: the duck shares the speaker with whatever app it
         * sits on, and its sounds are already evened out to a gentle loudness when rendered.
         */
        val VOLUMES = floatArrayOf(0f, 0.45f, 0.85f)

        /** A sound still waiting after this long has missed its moment, and is not played. */
        const val WAIT_MS = 3000L

        /** The registers each style was rendered in, by [PetState.voicePitch]. */
        val REGISTERS = arrayOf("low", "mid", "high")
    }
}
