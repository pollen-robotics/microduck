package io.github.samimreza.virtualduck

import android.content.Context
import android.view.SurfaceView
import io.github.samimreza.virtualduck.motion.Animator
import io.github.samimreza.virtualduck.pet.Brain
import io.github.samimreza.virtualduck.pet.PetState
import io.github.samimreza.virtualduck.pet.SoundVoice
import io.github.samimreza.virtualduck.render.DuckRenderer

/**
 * One duck on one surface: renderer, voice, animator and brain, stepped by a [Ticker]. The overlay
 * and the home screen each make one; [beforeRender] is where each puts its camera and windows to
 * match where the brain has just moved the duck.
 */
class DuckStage(
    context: Context,
    surface: SurfaceView,
    transparent: Boolean,
    assets: DuckAssets,
    state: PetState,
) {
    val renderer = DuckRenderer(surface, transparent, assets.glbBuffer(), assets.rig, assets.motions.joints)
    val voice = SoundVoice(context, state, assets.voices)
    val animator = Animator(assets.motions, renderer)
    val brain = Brain(assets.motions, animator, state, voice)

    var beforeRender: (() -> Unit)? = null

    /**
     * How many times faster than it is drawn the duck lives — 1 but for the end-to-end test, on an
     * emulator that draws a frame or two a second. Stepped a tenth of a second at a time at most.
     */
    var timeScale = 1f

    private val ticker = Ticker { dt, frameTimeNanos ->
        var left = dt * timeScale
        while (left > 0f) {
            val step = minOf(left, 0.1f)
            brain.update(step)
            left -= step
        }
        beforeRender?.invoke()
        renderer.render(frameTimeNanos)
        brain.frameRate
    }

    fun start() = ticker.start()

    /** Out of sight: no more frames, and nothing left singing from a duck nobody can see. */
    fun stop() {
        ticker.stop()
        brain.silence()
    }

    fun destroy() {
        ticker.stop()
        voice.release()
        renderer.destroy()
    }
}
