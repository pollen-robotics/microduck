package io.github.samimreza.virtualduck

import android.view.Choreographer

/**
 * A frame loop that runs only as often as the duck needs.
 *
 * [step] advances the duck by the time since the last step, draws it, and answers how many frames
 * a second the next stretch wants — thirty while it moves, six while it sleeps. Between steps the
 * loop is not called at all: the next callback is posted with a delay rather than woken on every
 * vsync to decide it has nothing to do, which on a 120 Hz screen is the difference between six
 * wakeups a second and a hundred and twenty.
 */
class Ticker(private val step: (dt: Float, frameTimeNanos: Long) -> Int) : Choreographer.FrameCallback {
    private val choreographer = Choreographer.getInstance()
    private var running = false
    private var last = 0L

    fun start() {
        if (running) return
        running = true
        last = 0L
        choreographer.postFrameCallback(this)
    }

    fun stop() {
        running = false
        choreographer.removeFrameCallback(this)
    }

    override fun doFrame(frameTimeNanos: Long) {
        if (!running) return
        // A long pause — the screen was off — is not time the duck should spend falling. The cap is
        // above the slowest frame it asks for (six a second, asleep), so a sleeping duck's time is
        // its own; DuckStage steps the brain no more than a tenth of a second at a time.
        val dt = if (last == 0L) 0f else ((frameTimeNanos - last) / 1e9f).coerceAtMost(0.25f)
        last = frameTimeNanos
        val fps = step(dt, frameTimeNanos).coerceIn(1, 60)
        // Aim a few milliseconds early: the callback lands on the next vsync after the delay.
        val delay = (1000L / fps - 6L).coerceAtLeast(0L)
        if (delay == 0L) choreographer.postFrameCallback(this) else choreographer.postFrameCallbackDelayed(this, delay)
    }
}
