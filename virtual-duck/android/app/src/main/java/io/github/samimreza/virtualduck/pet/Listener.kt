package io.github.samimreza.virtualduck.pet

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.pm.PackageManager
import android.media.AudioFormat
import android.media.AudioManager
import android.media.AudioRecord
import android.media.MediaRecorder
import android.os.Handler
import android.os.Looper
import kotlin.math.log10
import kotlin.math.sqrt

/**
 * The duck's ears: the microphone, read on a thread of its own and boiled down to one loudness a
 * frame for a [Conversation]. Samples go no further than that sum of squares — nothing is kept,
 * nothing is sent — and what the conversation hears is handed to the main thread.
 *
 * Deaf while the duck itself makes a sound ([deaf]), and while the phone plays anything else — a
 * video in the app underneath is not somebody talking to the duck. Every [onStart] it reports is
 * followed by an [onEnd] or an [onCancel], including when the ears are switched off mid-word.
 *
 * Runs only while the duck can be seen: the owner starts and stops it with the screen.
 */
class Listener(
    private val context: Context,
    private val onStart: () -> Unit,
    private val onEnd: (Conversation.Utterance) -> Unit,
    private val onCancel: () -> Unit,
) {
    /** Set while the duck is making a sound, so it does not answer itself. */
    @Volatile
    var deaf = false

    @Volatile
    private var running = false

    /** Bumped by every start and stop, so a thread still finishing its last read feeds nothing. */
    @Volatile
    private var generation = 0
    private val main = Handler(Looper.getMainLooper())
    private val audio = context.getSystemService(AudioManager::class.java)

    /** One for the ears' whole life, relearning the room each time they are switched on. */
    private val conversation = Conversation(1f / 50).apply {
        onStart = { main.post(this@Listener.onStart) }
        onEnd = { heard -> main.post { this@Listener.onEnd(heard) } }
        onCancel = { main.post(this@Listener.onCancel) }
    }

    val allowed: Boolean
        get() = context.checkSelfPermission(Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED

    @SuppressLint("MissingPermission")
    fun start() {
        if (running || !allowed) return
        running = true
        val mine = ++generation
        // Wherever it was last switched on, this may be another room: learn it again.
        synchronized(conversation) { conversation.restart() }
        Thread({
            var backoff = 1000L
            while (running && mine == generation) {
                // A session ends when the recorder cannot start, or dies with the audio server: it is
                // built again after a pause, growing to ten seconds, for as long as the ears are on.
                if (listen(mine)) backoff = 1000L
                if (!running || mine != generation) break
                Thread.sleep(backoff)
                backoff = (backoff * 2).coerceAtMost(10_000L)
            }
            // Switched off, stop() has already let go of anything half heard; a newer session's is its own.
            synchronized(conversation) { if (mine == generation) conversation.cancel() }
        }, "duck-ears").apply {
            priority = Thread.MIN_PRIORITY
            start()
        }
    }

    /** One recording session, until the ears are switched off or it fails. True if it heard anything. */
    @SuppressLint("MissingPermission")
    private fun listen(mine: Int): Boolean {
        val rate = 16_000
        val frame = rate / 50
        // In bytes: a few frames of 16-bit samples.
        val size = maxOf(AudioRecord.getMinBufferSize(rate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT), frame * 2 * 4)
        // VOICE_RECOGNITION: no automatic gain, so a level means the same thing all the time.
        val record = runCatching {
            AudioRecord(MediaRecorder.AudioSource.VOICE_RECOGNITION, rate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, size)
        }.getOrNull() ?: return false
        var heard = false
        try {
            if (record.state != AudioRecord.STATE_INITIALIZED) return false
            // Another app may have the microphone: then this session ends and the next tries again.
            runCatching { record.startRecording() }
            if (record.recordingState != AudioRecord.RECORDSTATE_RECORDING) return false
            val samples = ShortArray(frame)
            var failures = 0
            var frames = 0
            var others = false
            while (running && mine == generation) {
                val n = record.read(samples, 0, frame)
                if (n == AudioRecord.ERROR_DEAD_OBJECT) break
                if (n <= 0) {
                    if (++failures > 100) break
                    Thread.sleep(50)
                    continue
                }
                failures = 0
                heard = true
                // Five times a second: is the phone playing something — a video, music, a call? Not
                // while the duck itself speaks, which plays through the same stream and would keep it
                // deaf for a beat longer than its own sound.
                if (deaf) {
                    others = false
                } else if (frames++ % 10 == 0) {
                    others = audio?.isMusicActive == true
                }
                var sum = 0.0
                for (i in 0 until n) sum += samples[i].toDouble() * samples[i]
                val rms = sqrt(sum / n) / 32768.0
                synchronized(conversation) {
                    if (mine == generation) conversation.frame((20 * log10(rms + 1e-9)).toFloat(), deaf || others)
                }
            }
        } finally {
            runCatching { record.stop() }
            record.release()
            // Anything half heard when the session ended is not going to be finished.
            synchronized(conversation) { if (mine == generation) conversation.cancel() }
        }
        return heard
    }

    fun stop() {
        running = false
        generation++
        synchronized(conversation) { conversation.cancel() }
    }
}
