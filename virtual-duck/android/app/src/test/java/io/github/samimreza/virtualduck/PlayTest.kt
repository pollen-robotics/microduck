package io.github.samimreza.virtualduck

import io.github.samimreza.virtualduck.motion.Animator
import io.github.samimreza.virtualduck.motion.wrapAngle
import io.github.samimreza.virtualduck.pet.Activity
import io.github.samimreza.virtualduck.pet.Ball
import io.github.samimreza.virtualduck.pet.Brain
import io.github.samimreza.virtualduck.pet.Conversation
import io.github.samimreza.virtualduck.pet.PetState
import io.github.samimreza.virtualduck.pet.ShakeDetector
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sin
import kotlin.random.Random

/**
 * What the owner asks of the duck must happen, whatever its mood and wherever it stands: a ball
 * thrown to a duck that has had all the fun it wants is still kicked, seeds thrown to a full one
 * still eaten, a ball against the wall still reached. Driven with the recorded motions, in a floor
 * as narrow as the home screen's.
 */
class PlayTest {
    private val dt = 1f / 30

    private fun brain(state: PetState = PetState(), seed: Int = 3, day: Long = 20_000L): Pair<Brain, FakeVoice> {
        val motions = recordedLibrary()
        val voice = FakeVoice()
        var today = day
        val brain = Brain(motions, Animator(motions, FakeBody()), state, voice, Random(seed)) { today }
        brain.left = 0.12f
        brain.right = 0.6f
        brain.animator.place(0.36f, 0f, Brain.VIEWER)
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

    private fun content() = PetState(food = 1f, energy = 1f, joy = 1f, love = 1f)

    @Test
    fun `a ball the owner throws is kicked even by a duck with all the fun it wants`() {
        val (brain, _) = brain(content())
        brain.throwBall()
        assertTrue("never kicked", run(brain, 30f) { brain.state.kicks > 0 })
    }

    @Test
    fun `a ball resting against the wall is still kicked`() {
        val (brain, _) = brain(content())
        brain.throwBall()
        run(brain, 0.5f)
        val ball = brain.ball!!
        ball.x = brain.right
        ball.vx = 0f
        assertTrue("never kicked", run(brain, 30f) { brain.state.kicks > 0 })
    }

    @Test
    fun `seeds the owner throws are eaten even by a full duck`() {
        val (brain, _) = brain(content())
        brain.feed()
        brain.feed()
        assertTrue("seeds left: ${brain.pellets}", run(brain, 45f) { brain.pellets.isEmpty() })
        assertEquals(2, brain.state.pelletsEaten)
    }

    @Test
    fun `asked to dance mid-walk, it stops and dances`() {
        val (brain, _) = brain()
        brain.come(0.58f)
        assertTrue(run(brain, 5f) { brain.activity == Activity.WALKING })
        brain.dance()
        assertTrue("never danced", run(brain, 10f) { brain.activity == Activity.DANCING })
    }

    @Test
    fun `asked to roll, it rolls`() {
        val (brain, _) = brain()
        brain.trick()
        assertTrue(run(brain, 5f) { brain.activity == Activity.ROLLING })
    }

    @Test
    fun `talked to, it answers with as many sounds as it heard syllables`() {
        val (brain, voice) = brain()
        brain.listen()
        brain.heard(3, listOf(0.2f, 0.25f))
        run(brain, 4f)
        val answer = voice.said.filter { it in setOf("chirp", "peck", "coo", "greet", "inquire") }
        assertEquals("said ${voice.said}", 3, answer.size)
        assertEquals(1, brain.state.chats)
    }

    @Test
    fun `a jump to a higher floor lands on its feet`() {
        val (brain, _) = brain()
        brain.jump(0.5f, from = -0.3f)
        var fell = false
        assertTrue(run(brain, 5f) {
            fell = fell || brain.activity == Activity.FALLING
            brain.activity == Activity.IDLE || brain.activity == Activity.TURNING
        })
        assertFalse(fell)
        assertEquals(0f, brain.lift, 1e-6f)
        assertEquals(0.5f, brain.animator.floorX, 0.08f)
    }

    @Test
    fun `a kindness a day keeps the streak, and friendship levels up`() {
        val state = PetState()
        assertTrue(state.cared(100))
        assertFalse(state.cared(100))
        assertTrue(state.cared(101))
        assertEquals(2, state.streak)
        assertTrue(state.cared(105))
        assertEquals("a missed day starts again", 1, state.streak)
        assertEquals(1, state.level)
        assertTrue(state.gain(50))
        assertEquals(2, state.level)
    }

    // ------------------------------------------------------------ what it was found ignoring them for

    @Test
    fun `a sound that never turned into speech does not stop it eating`() {
        val (brain, _) = brain(content())
        brain.listen()
        brain.feed()
        assertTrue("seeds left: ${brain.status()}", run(brain, 45f) { brain.pellets.isEmpty() })
    }

    @Test
    fun `a sound starting while it walks to the ball does not stop it kicking`() {
        val (brain, _) = brain(content())
        brain.throwBall()
        assertTrue(run(brain, 10f) { brain.activity == Activity.WALKING || brain.activity == Activity.TURNING })
        brain.listen()
        assertTrue("never kicked: ${brain.status()}", run(brain, 30f) { brain.state.kicks > 0 })
    }

    @Test
    fun `it stops listening to a sound that never ends`() {
        val (brain, _) = brain(content())
        brain.listen()
        run(brain, Brain.LISTEN_MAX + 1f)
        assertEquals(false, brain.status()["listening"])
    }

    @Test
    fun `seeds next to it by a wall are eaten, from every side`() {
        for (wall in listOf(-1f, 1f)) {
            for (from in 0..7) for (to in 0..13) for (heading in listOf(0f, PI.toFloat())) {
                val (brain, _) = brain(content())
                val edge = if (wall < 0f) brain.left else brain.right
                brain.animator.place(edge - wall * (0.005f + from * 0.02f), 0f, heading)
                brain.feed()
                brain.pellets[0] = edge - wall * (Brain.SEED_MARGIN + to * 0.02f)
                assertTrue(
                    "seed at ${brain.pellets} from ${brain.animator.floorX} facing $heading: ${brain.status()}",
                    run(brain, 40f) { brain.pellets.isEmpty() },
                )
            }
        }
    }

    @Test
    fun `a ball next to it by a wall is kicked`() {
        for (wall in listOf(-1f, 1f)) {
            for (from in 0..5) for (to in 0..6) {
                val (brain, _) = brain(content())
                val edge = if (wall < 0f) brain.left else brain.right
                brain.animator.place(edge - wall * (0.005f + from * 0.03f), 0f, if (wall < 0f) PI.toFloat() else 0f)
                brain.throwBall()
                run(brain, 0.1f)
                val ball = brain.ball!!
                ball.x = edge - wall * to * 0.03f
                ball.z = Ball.RADIUS
                ball.vx = 0f
                ball.vz = 0f
                assertTrue("ball at ${ball.x} from ${brain.animator.floorX}: ${brain.status()}", run(brain, 40f) { brain.state.kicks > 0 })
            }
        }
    }

    @Test
    fun `seeds left beyond a floor that shrank are still eaten`() {
        val (brain, _) = brain(content())
        brain.feed()
        brain.feed()
        brain.pellets[0] = brain.right + 0.4f
        brain.pellets[1] = brain.right + 0.1f
        brain.animator.place(brain.right - 0.02f, 0f, 0f)
        assertTrue("seeds left: ${brain.status()}", run(brain, 60f) { brain.pellets.isEmpty() })
    }

    @Test
    fun `a rescaled floor carries its seeds and ball with it`() {
        val (brain, _) = brain(content())
        brain.feed()
        brain.throwBall()
        val seed = brain.pellets[0]
        val ball = brain.ball!!.x
        brain.rescale(0.5f)
        assertEquals(seed * 0.5f, brain.pellets[0], 1e-6f)
        assertEquals(ball * 0.5f, brain.ball!!.x, 1e-6f)
    }

    @Test
    fun `a sleepy duck does not wander off while the owner's ball is still in the air`() {
        val (brain, _) = brain(PetState(food = 1f, energy = 0.12f, joy = 1f, love = 1f))
        brain.throwBall()
        var slept = false
        assertTrue("never kicked: ${brain.status()}", run(brain, 30f) {
            slept = slept || brain.activity == Activity.SLEEPING || brain.activity == Activity.SITTING
            brain.state.kicks > 0
        })
        assertFalse("sat or slept first", slept)
    }

    @Test
    fun `a kick cut short lets go of the ball, and it is kicked again`() {
        val (brain, _) = brain(content())
        brain.throwBall()
        assertTrue(run(brain, 30f) { brain.activity == Activity.KICKING && brain.ball!!.scripted })
        brain.grab(brain.animator.floorX)
        brain.release(0f, 0f)
        assertFalse(brain.ball!!.scripted)
        brain.throwBall()
        val kicks = brain.state.kicks
        assertTrue("never kicked again: ${brain.status()}", run(brain, 40f) { brain.state.kicks > kicks })
    }

    @Test
    fun `a resting ball stays resting however long the frames`() {
        val (brain, _) = brain(content())
        brain.throwBall()
        val ball = brain.ball!!
        var t = 0f
        while (!ball.resting && t < 10f) {
            brain.update(0.1f)
            t += 0.1f
        }
        repeat(30) {
            brain.update(0.1f)
            if (brain.activity != Activity.KICKING) assertTrue("bounced on its own at frame $it", ball.resting)
        }
    }

    @Test
    fun `on the charger it naps only after eating the seeds it was given`() {
        val (brain, _) = brain(PetState(food = 1f, energy = 0.5f, joy = 1f, love = 1f))
        brain.feed()
        brain.feed()
        run(brain, 1f)
        brain.charging(true)
        var sleptEarly = false
        assertTrue("seeds left: ${brain.status()}", run(brain, 60f) {
            if (brain.pellets.isNotEmpty() && brain.activity == Activity.SLEEPING) sleptEarly = true
            brain.pellets.isEmpty()
        })
        assertFalse("fell asleep beside the seeds", sleptEarly)
        assertTrue("never napped", run(brain, 20f) { brain.activity == Activity.SLEEPING })
        brain.charging(false)
        assertTrue("never woke", run(brain, 10f) { brain.activity != Activity.SLEEPING })
    }

    @Test
    fun `food in the middle of a song stops the song and is eaten`() {
        val (brain, voice) = brain(PetState(food = 1f, energy = 1f, joy = 1f, love = 1f, xp = 1000))
        brain.sing()
        assertTrue(run(brain, 5f) { brain.activity == Activity.DANCING })
        brain.feed()
        assertTrue("hush" in voice.said)
        assertTrue("seeds left: ${brain.status()}", run(brain, 30f) { brain.pellets.isEmpty() })
    }

    @Test
    fun `a flicked ball wakes it`() {
        // Not rested: a duck with all the energy it wants wakes up again at once.
        val (brain, _) = brain(PetState(food = 1f, energy = 0.6f, joy = 1f, love = 1f))
        brain.throwBall()
        run(brain, 0.1f)
        brain.toggleSleep()
        assertTrue(run(brain, 10f) { brain.activity == Activity.SLEEPING })
        val kicks = brain.state.kicks
        brain.flickBall(0.5f)
        assertTrue("never kicked: ${brain.status()}", run(brain, 40f) { brain.state.kicks > kicks })
    }

    @Test
    fun `the two screens' ducks agree on whether it is asleep`() {
        val state = PetState(energy = 0.5f)
        val (overlay, _) = brain(state)
        overlay.toggleSleep()
        assertTrue(run(overlay, 10f) { overlay.activity == Activity.SLEEPING })
        val (home, _) = brain(state)
        assertEquals(Activity.SLEEPING, home.activity)
        home.poke(home.animator.floorX, 0.2f)
        assertFalse(state.asleep)
        overlay.sync()
        assertTrue("overlay still asleep", run(overlay, 10f) { overlay.activity == Activity.IDLE })
    }

    @Test
    fun `a sleeping duck let out on the screen stays asleep`() {
        val (brain, _) = brain(PetState(asleep = true, energy = 0.5f))
        brain.dropIn()
        run(brain, 3f)
        assertEquals(Activity.SLEEPING, brain.activity)
        brain.grab(brain.animator.floorX)
        assertFalse("picked up, it is awake", brain.state.asleep)
    }

    @Test
    fun `called to the edge of its floor, it stays on it`() {
        val (brain, _) = brain(content())
        brain.animator.place(brain.right - 0.07f, 0f, 0f)
        brain.come(brain.right)
        run(brain, 8f)
        assertTrue("at ${brain.animator.floorX}, right is ${brain.right}", brain.animator.floorX <= brain.right + 0.01f)
    }

    @Test
    fun `after a roll it finds its way back to its floor line`() {
        val (brain, _) = brain(content())
        brain.trick()
        assertTrue(run(brain, 5f) { brain.activity == Activity.ROLLING })
        run(brain, 20f) { brain.activity == Activity.IDLE }
        run(brain, 15f)
        assertEquals(0f, brain.animator.floorY, 0.02f)
    }

    // ------------------------------------------------------------ what review of those fixes found

    @Test
    fun `a jump as a dance ends still lands`() {
        val (brain, _) = brain(content())
        brain.dance()
        run(brain, 5.8f)
        brain.jump(0.4f, from = -0.2f)
        run(brain, 1f)
        assertEquals(0f, brain.lift, 1e-3f)
    }

    @Test
    fun `told to sleep while it is getting up, it sits back down and sleeps`() {
        val (brain, _) = brain(PetState(asleep = true, energy = 0.5f))
        brain.poke(brain.animator.floorX, 0.2f)
        run(brain, 1f)
        brain.toggleSleep()
        run(brain, 8f)
        assertEquals(Activity.SLEEPING, brain.activity)
        assertTrue(brain.state.asleep)
    }

    @Test
    fun `a sound starting while it walks where it was called does not stop it`() {
        val (brain, _) = brain(content())
        brain.animator.place(0.2f, 0f, Brain.VIEWER)
        brain.come(0.5f)
        assertTrue(run(brain, 5f) { brain.activity == Activity.WALKING })
        brain.listen()
        run(brain, 10f) { brain.activity == Activity.IDLE }
        assertEquals(0.5f, brain.animator.floorX, 0.09f)
    }

    @Test
    fun `a sound starting while it turns to roll does not stop the roll`() {
        val (brain, _) = brain(content())
        brain.animator.place(0.36f, 0f, 0f)
        brain.trick()
        assertTrue(run(brain, 2f) { brain.activity == Activity.TURNING })
        brain.listen()
        assertTrue("never rolled: ${brain.status()}", run(brain, 8f) { brain.activity == Activity.ROLLING })
    }

    private fun singing(): Pair<Brain, FakeVoice> {
        val pair = brain(PetState(food = 1f, energy = 0.5f, joy = 1f, love = 1f, xp = 1000))
        pair.first.sing()
        assertTrue(run(pair.first, 5f) { pair.first.activity == Activity.DANCING })
        return pair
    }

    @Test
    fun `a song cut short by sleep stops`() {
        val (brain, voice) = singing()
        brain.toggleSleep()
        assertTrue("hush" in voice.said)
        assertTrue(run(brain, 10f) { brain.activity == Activity.SLEEPING })
    }

    @Test
    fun `a song cut short by a tumble stops`() {
        val (brain, voice) = singing()
        brain.shaken(1f)
        assertTrue("hush" in voice.said)
    }

    @Test
    fun `a song stops when the duck is put out of sight`() {
        val (brain, voice) = singing()
        brain.silence()
        assertTrue("hush" in voice.said)
        assertTrue("still dancing", run(brain, 4f) { brain.activity != Activity.DANCING })
    }

    @Test
    fun `asked again and again while getting up, it still gets up`() {
        val (brain, _) = brain(PetState(asleep = true, energy = 0.5f))
        brain.poke(brain.animator.floorX, 0.2f)
        var walked = false
        repeat(8) {
            brain.come(0.55f)
            walked = walked || run(brain, 1f) { brain.activity == Activity.WALKING || brain.activity == Activity.TURNING }
        }
        assertTrue("never got up: ${brain.status()}", walked)
    }

    @Test
    fun `a duck busy when the other screen's duck fell asleep goes to sleep when it is free`() {
        val (brain, _) = brain(PetState(energy = 0.5f))
        brain.dance()
        assertTrue(run(brain, 3f) { brain.activity == Activity.DANCING })
        brain.state.asleep = true
        brain.sync()
        assertTrue("still awake: ${brain.status()}", run(brain, 15f) { brain.activity == Activity.SLEEPING })
    }

    @Test
    fun `a floor narrower than a step does not crash it`() {
        val (brain, _) = brain(content())
        brain.left = 0.12f
        brain.right = 0.17f
        brain.animator.place(0.3f, 0f, 0f)
        brain.feed()
        brain.throwBall()
        run(brain, 30f)
    }

    @Test
    fun `a floor rescaled mid-kick does not fling the ball`() {
        val (brain, _) = brain(content())
        brain.throwBall()
        assertTrue(run(brain, 30f) { brain.activity == Activity.KICKING && brain.ball!!.scripted })
        brain.animator.place(brain.animator.floorX * 0.75f, brain.animator.floorY, brain.animator.facing)
        brain.rescale(0.75f)
        run(brain, 0.1f)
        assertTrue("ball at ${brain.ball!!.vx} m/s", abs(brain.ball!!.vx) < 1f)
    }

    /**
     * Ten minutes of somebody poking at it: every action in random order, at random moments. It must
     * never throw, never leave its floor for long, and never be stuck in one busy thing — only
     * sleeping and sitting are allowed to last.
     */
    @Test
    fun `ten minutes of random play leaves it working`() {
        val (brain, _) = brain(PetState(energy = 0.8f))
        val random = Random(11)
        var stuckFor = 0f
        var last = brain.activity
        var t = 0f
        while (t < 600f) {
            if (random.nextFloat() < dt / 4f) {
                when (random.nextInt(11)) {
                    0 -> brain.feed()
                    1 -> brain.throwBall()
                    2 -> brain.dance()
                    3 -> brain.trick()
                    4 -> brain.toggleSleep()
                    5 -> brain.poke(brain.animator.floorX, 0.2f)
                    6 -> brain.stroke(brain.animator.floorX, 0.25f)
                    7 -> brain.come(0.12f + random.nextFloat() * 0.48f)
                    8 -> if (random.nextBoolean()) brain.heard(1 + random.nextInt(5), listOf(0.2f, 0.3f)) else brain.listen()
                    9 -> brain.sing()
                    else -> {
                        brain.grab(brain.animator.floorX)
                        repeat(10) { brain.carry(brain.animator.floorX + 0.01f * it, 0.2f, dt) }
                        brain.release(random.nextFloat() * 4f - 2f, random.nextFloat() * 2f)
                    }
                }
            }
            brain.update(dt)
            t += dt
            val a = brain.activity
            stuckFor = if (a == last && a != Activity.IDLE && a != Activity.SLEEPING && a != Activity.SITTING) stuckFor + dt else 0f
            last = a
            assertTrue("stuck in $a for ${stuckFor}s at t=$t", stuckFor < 40f)
            assertTrue("off its floor at ${brain.animator.floorX}", brain.animator.floorX in (brain.left - 0.35f)..(brain.right + 0.35f))
        }
        // And after all that, the owner's seeds are still eaten.
        brain.feed()
        assertTrue("seeds left after the soak: ${brain.status()}", run(brain, 60f) { brain.pellets.isEmpty() })
    }
}

class ListenTest {
    /** Feeds a [Conversation] a quiet room, then [speech], then quiet again; returns what it heard. */
    private fun hear(speech: List<Float>): List<Conversation.Utterance> {
        val heard = mutableListOf<Conversation.Utterance>()
        val c = Conversation().apply { onEnd = { heard += it } }
        repeat(100) { c.frame(-62f + (it % 3)) }
        speech.forEach { c.frame(it) }
        repeat(60) { c.frame(-61f) }
        return heard
    }

    /** [n] syllables: rises to -20 dBFS, dips to -38 between them, 20 ms frames. */
    private fun syllables(n: Int): List<Float> = (0 until n).flatMap { List(8) { -20f } + List(6) { -38f } }

    @Test
    fun `three spoken syllables are heard as three`() {
        val heard = hear(syllables(3))
        assertEquals(1, heard.size)
        assertEquals(3, heard[0].syllables)
        assertEquals(2, heard[0].gaps.size)
    }

    @Test
    fun `a click is not speech`() {
        assertTrue(hear(List(3) { -15f }).isEmpty())
    }

    @Test
    fun `a television that never stops is not someone talking to the duck`() {
        assertTrue(hear(syllables(40)).isEmpty())
    }

    /** Every start it reports ends in exactly one end or cancel, whatever the sound. */
    private fun balanced(levels: List<Pair<Float, Boolean>>, c: Conversation = Conversation()): Pair<Int, Int> {
        var open = 0
        var starts = 0
        c.onStart = { open++; starts++ }
        c.onEnd = { open-- }
        c.onCancel = { open-- }
        repeat(100) { c.frame(-62f) }
        levels.forEach { (db, deaf) ->
            c.frame(db, deaf)
            assertTrue("$open open", open in 0..1)
        }
        c.cancel()
        assertEquals(0, open)
        return starts to open
    }

    @Test
    fun `a sound too short to be speech is called off`() {
        val (starts, _) = balanced(List(8) { -20f to false } + List(40) { -62f to false })
        assertEquals(1, starts)
    }

    @Test
    fun `a sound cut off by the duck's own voice is called off`() {
        balanced(List(20) { -20f to false } + List(20) { -20f to true } + List(40) { -62f to false })
    }

    @Test
    fun `a television is called off, and then it is the room`() {
        val tv = (0 until 2000).map { (if (it % 14 < 8) -24f else -30f) to false }
        val (starts, _) = balanced(tv)
        assertTrue("restarted $starts times", starts <= 2)
    }

    @Test
    fun `whatever it hears, it is never left listening`() {
        val random = Random(5)
        val levels = (0 until 20_000).map { (-70f + random.nextFloat() * 55f) to (random.nextFloat() < 0.05f) }
        balanced(levels)
    }

    @Test
    fun `just after its own voice, the echo is not somebody talking`() {
        val heard = mutableListOf<Conversation.Utterance>()
        val c = Conversation().apply { onEnd = { heard += it } }
        repeat(100) { c.frame(-62f) }
        repeat(30) { c.frame(-20f, deaf = true) }
        // The room still ringing for a moment after the speaker stopped.
        repeat(15) { c.frame(-25f) }
        repeat(60) { c.frame(-61f) }
        assertTrue(heard.isEmpty())
    }

    @Test
    fun `switched on in a louder room, it learns the room rather than hearing it as talk`() {
        var starts = 0
        val c = Conversation().apply { onStart = { starts++ } }
        repeat(100) { c.frame(-62f) }
        c.restart()
        repeat(600) { c.frame(-40f + (it % 3)) }
        assertEquals(0, starts)
    }

    @Test
    fun `it does not hear itself`() {
        val heard = mutableListOf<Conversation.Utterance>()
        val c = Conversation().apply { onEnd = { heard += it } }
        repeat(100) { c.frame(-62f) }
        syllables(3).forEach { c.frame(it, deaf = true) }
        repeat(60) { c.frame(-61f) }
        assertTrue(heard.isEmpty())
    }
}

class ShakeTest {
    @Test
    fun `three hard jolts are a shake, a walk is not`() {
        var shakes = 0
        val d = ShakeDetector { shakes++ }
        var t = 0f
        // Walking: a gentle bounce around gravity.
        repeat(300) {
            d.sample(t, 0.5f, 9.81f + 2f * sin(t * 12f), 0.3f)
            t += 0.02f
        }
        assertEquals(0, shakes)
        repeat(3) {
            d.sample(t, 22f, 9.81f, 0f)
            t += 0.15f
        }
        assertEquals(1, shakes)
        // And not again straight away: one shake, one tumble.
        repeat(3) {
            d.sample(t, -22f, 9.81f, 0f)
            t += 0.15f
        }
        assertEquals(1, shakes)
    }
}

@Suppress("unused")
private fun angleClose(a: Float, b: Float) = abs(wrapAngle(a - b)) < 0.5f
