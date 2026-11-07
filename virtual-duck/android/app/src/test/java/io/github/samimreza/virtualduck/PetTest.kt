package io.github.samimreza.virtualduck

import io.github.samimreza.virtualduck.motion.Animator
import io.github.samimreza.virtualduck.pet.Activity
import io.github.samimreza.virtualduck.pet.Brain
import io.github.samimreza.virtualduck.pet.Mood
import io.github.samimreza.virtualduck.pet.PetState
import io.github.samimreza.virtualduck.motion.wrapAngle
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.random.Random

class PetStateTest {
    @Test
    fun `needs drain while awake and never past the floor`() {
        val pet = PetState(food = 0.5f)
        pet.elapse(3600f * 100)
        assertEquals(PetState.FLOOR, pet.food, 1e-6f)
        assertEquals(PetState.FLOOR, pet.love, 1e-6f)
    }

    @Test
    fun `sleep brings energy back`() {
        val pet = PetState(energy = 0.2f, asleep = true)
        pet.elapse(3600f)
        assertTrue(pet.energy > 0.35f)
    }

    @Test
    fun `time away is aged as rest, and only so far back`() {
        // A month away must not cost more than a day: nobody should come back to a wreck.
        val pet = PetState(energy = 0.3f, food = 0.9f, updatedAtMs = 1_000L)
        pet.catchUp(1_000L + 30L * 24 * 3600 * 1000)
        assertTrue("energy ${pet.energy}", pet.energy > 0.9f)
        assertTrue("food ${pet.food}", pet.food > PetState.FLOOR)
        assertFalse(pet.asleep)
    }

    @Test
    fun `mood follows the emptiest need`() {
        assertEquals(Mood.HUNGRY, PetState(food = 0.1f).mood)
        assertEquals(Mood.SLEEPY, PetState(energy = 0.1f, food = 0.1f).mood)
        assertEquals(Mood.HAPPY, PetState(food = 0.9f, energy = 0.9f, joy = 0.9f, love = 0.9f).mood)
    }

    @Test
    fun `it survives being saved`() {
        val pet = PetState(name = "Quackers", voice = "squeaky", voicePitch = 2, food = 0.4f, soundLevel = 0, sizeDp = 200, pelletsEaten = 7, xp = 120, streak = 4, floorLevel = 0.5f)
        val back = PetState.fromJson(pet.toJson())
        assertEquals("Quackers", back.name)
        assertEquals("squeaky", back.voice)
        assertEquals(2, back.voicePitch)
        assertEquals(0.4f, back.food, 1e-6f)
        assertTrue(back.muted)
        assertEquals(120, back.xp)
        assertEquals(4, back.streak)
        assertEquals(0.5f, back.floorLevel, 1e-6f)
        assertEquals(200, back.sizeDp)
        assertEquals(7, back.pelletsEaten)
    }

    @Test
    fun `a duck saved before sound had levels keeps being quiet`() {
        assertTrue(PetState.fromJson("{\"muted\": true}").muted)
        assertEquals(1, PetState.fromJson("{}").soundLevel)
    }

    @Test
    fun `a duck saved with one of the old numbered voices speaks in the default one`() {
        val back = PetState.fromJson("{\"voice_seed\": 23, \"xp\": 80}")
        assertEquals(PetState.DEFAULT_VOICE, back.voice)
        assertEquals(1, back.voicePitch)
        assertEquals(80, back.xp)
    }
}

/**
 * The brain, driven through whole behaviours with the motions recorded from the real policies —
 * the only motions it will ever play, so the only ones worth testing it against.
 */
class BrainTest {
    private val dt = 1f / 30

    private fun brain(state: PetState = PetState(), seed: Int = 7): Pair<Brain, FakeVoice> {
        val motions = recordedLibrary()
        val voice = FakeVoice()
        val brain = Brain(motions, Animator(motions, FakeBody()), state, voice, Random(seed))
        brain.left = 0.1f
        brain.right = 1.6f
        brain.animator.place(0.8f, 0f, Brain.VIEWER)
        return brain to voice
    }

    private fun run(brain: Brain, seconds: Float, until: () -> Boolean = { false }): Boolean {
        var t = 0f
        while (t < seconds) {
            brain.update(dt)
            t += dt
            if (until()) return true
        }
        return false
    }

    @Test
    fun `a hungry duck walks to a seed and eats it`() {
        val (brain, voice) = brain(PetState(food = 0.4f))
        brain.feed()
        val seed = brain.pellets.single()
        assertTrue("never ate", run(brain, 30f) { brain.pellets.isEmpty() })
        assertEquals(1, brain.state.pelletsEaten)
        assertTrue(voice.said.contains("peck"))
        assertTrue("ate from ${brain.animator.floorX}, seed at $seed", abs(brain.animator.floorX - seed) < 0.2f)
    }

    @Test
    fun `a thrown duck lands, falls over and gets back up`() {
        val (brain, voice) = brain()
        brain.grab(0.8f)
        repeat(10) { brain.carry(0.8f + it * 0.01f, 0.4f, dt) }
        brain.release(2.5f, 1.5f)
        var fell = false
        assertTrue(
            "still not up",
            run(brain, 12f) {
                fell = fell || brain.activity == Activity.FALLING
                fell && brain.activity == Activity.IDLE
            },
        )
        assertEquals(0f, brain.lift, 1e-6f)
        assertTrue(voice.said.contains("alarm"))
        assertTrue("left the floor: ${brain.animator.floorX}", brain.animator.floorX in 0.05f..1.65f)
    }

    @Test
    fun `a bored duck chases a ball and kicks it`() {
        val (brain, _) = brain(PetState(joy = 0.3f))
        brain.throwBall()
        var kicked = false
        val ball = { brain.ball!! }
        run(brain, 3f)
        val restedAt = ball().x
        assertTrue(
            "never kicked",
            run(brain, 40f) {
                kicked = kicked || brain.activity == Activity.KICKING
                kicked && brain.activity == Activity.IDLE
            },
        )
        run(brain, 3f)
        assertTrue("ball went from $restedAt to ${ball().x}", abs(ball().x - restedAt) > 0.1f)
    }

    @Test
    fun `turning to the viewer ends facing the viewer`() {
        val (brain, _) = brain()
        brain.animator.place(0.8f, 0f, 0f)
        // Asking for a stroke is a turn to the viewer; a lonely duck asks.
        brain.state.love = 0.1f
        brain.state.food = 0.9f
        brain.state.energy = 0.9f
        assertTrue(run(brain, 15f) { brain.bubble == "❤️" })
        assertTrue("facing ${brain.animator.facing}", abs(wrapAngle(brain.animator.facing - Brain.VIEWER)) < 0.6f)
    }

    @Test
    fun `a dance asked of a sleeping duck wakes it, stands it up, then dances`() {
        // A button pressed while the duck is busy must still do what it says, not nothing.
        val (brain, _) = brain()
        brain.toggleSleep()
        assertTrue(run(brain, 10f) { brain.activity == Activity.SLEEPING })
        brain.dance()
        assertTrue("never danced", run(brain, 15f) { brain.activity == Activity.DANCING })
        assertFalse(brain.state.asleep)
    }

    @Test
    fun `a sleepy duck goes to sleep and a poke wakes it`() {
        val (brain, _) = brain(PetState(energy = 0.1f))
        assertTrue(run(brain, 15f) { brain.activity == Activity.SLEEPING })
        assertTrue(brain.state.asleep)
        brain.poke(0.8f, 0.2f)
        assertTrue(run(brain, 10f) { brain.activity == Activity.IDLE })
        assertFalse(brain.state.asleep)
    }
}
