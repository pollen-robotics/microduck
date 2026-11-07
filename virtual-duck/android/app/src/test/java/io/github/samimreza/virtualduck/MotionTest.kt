package io.github.samimreza.virtualduck

import io.github.samimreza.virtualduck.motion.Animator
import io.github.samimreza.virtualduck.motion.Ground
import io.github.samimreza.virtualduck.motion.TRUNK
import io.github.samimreza.virtualduck.motion.Vec2
import io.github.samimreza.virtualduck.motion.headingOf
import io.github.samimreza.virtualduck.motion.wrapAngle
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class GeometryTest {
    @Test
    fun `a ground frame composed with its inverse is the origin`() {
        // place() leans on this: moving the duck is target · here⁻¹ · ground.
        val g = Ground(0.3f, -0.2f, 1.1f)
        val back = g * g.inverse()
        assertEquals(0f, back.x, 1e-5f)
        assertEquals(0f, back.y, 1e-5f)
        assertEquals(0f, wrapAngle(back.yaw), 1e-5f)
    }

    @Test
    fun `turning about a point leaves that point where it was`() {
        val pivot = Vec2(0.5f, 0.1f)
        val g = Ground(0.2f, 0f, 0.3f)
        val turned = g.turnedAbout(pivot, 0.8f)
        // The pivot, expressed in g's frame, maps to the same room point through the turned frame.
        val local = g.inverse().apply(pivot)
        val after = turned.apply(local)
        assertEquals(pivot.x, after.x, 1e-5f)
        assertEquals(pivot.y, after.y, 1e-5f)
    }
}

class MotionLibraryTest {
    @Test
    fun `a loop's last frame blends toward its end frame, not back to its first`() {
        // Without `end`, the wrap would interpolate from the last frame back to the first and the
        // walk would jerk backwards once a stride.
        val walk = syntheticLibrary(stride = 0.05f, period = 20).clip("walk")
        val out = FloatArray(walk.frameSize)
        walk.sampleInto(walk.duration - 0.01f, out)
        assertTrue("x=${out[0]}", out[0] > 0.0475f)
    }

    @Test
    fun `a loop's cycle is its end frame's ground`() {
        val walk = syntheticLibrary(stride = 0.05f).clip("walk")
        assertEquals(0.05f, walk.cycle.x, 1e-6f)
        assertEquals(0f, walk.cycle.yaw, 1e-6f)
    }

    @Test
    fun `quaternions blend the short way round`() {
        // q and -q are the same rotation; blending toward -q must not swing through 360°.
        val turn = List(2) { i ->
            FloatArray(7 + JOINTS.size).also {
                val s = if (i == 0) 1f else -1f
                it[3] = s * cos(0.1f)
                it[6] = s * sin(0.1f)
            }
        }
        val clip = syntheticLibrary(extra = mapOf("flip" to turn)).clip("flip")
        val out = FloatArray(clip.frameSize)
        clip.sampleInto(0.01f, out)
        assertEquals(0.2f, headingOf(out[3], out[4], out[5], out[6]), 1e-4f)
    }

    @Test
    fun `the recorded walk goes forward and its loop closes`() {
        val library = recordedLibrary()
        val walk = library.clip("walk")
        assertTrue("stride ${walk.cycle.x}", walk.cycle.x in 0.03f..0.08f)
        assertTrue("drift ${walk.cycle.yaw}", abs(walk.cycle.yaw) < 0.05f)
        val peck = library.clip("peck")
        assertTrue("peck has a bite", peck.events.containsKey("bite_s") && peck.events.containsKey("reach_m"))
    }
}

class AnimatorTest {
    private val dt = 1f / 30

    @Test
    fun `a walk loop carries the duck by its stride every cycle`() {
        val body = FakeBody()
        val animator = Animator(syntheticLibrary(stride = 0.05f, period = 20), body)
        animator.play("walk", 0f)
        repeat((10 * 0.4f / dt).toInt()) { animator.advance(dt) }
        // Ten cycles of 0.4 s, 5 cm each.
        assertEquals(0.5f, animator.floorX, 0.03f)
    }

    @Test
    fun `a clip starts where the duck stands, not at the origin`() {
        val body = FakeBody()
        val animator = Animator(syntheticLibrary(), body)
        animator.place(0.7f, 0.2f, 1.0f)
        animator.play("walk", 0f)
        assertEquals(0.7f, body.frame[0], 1e-4f)
        assertEquals(0.2f, body.frame[1], 1e-4f)
        assertEquals(1.0f, headingOf(body.frame[3], body.frame[4], body.frame[5], body.frame[6]), 1e-4f)
    }

    @Test
    fun `steering turns the walk at the rate asked`() {
        val animator = Animator(syntheticLibrary(), FakeBody())
        animator.play("walk", 0f)
        animator.steerRate = 0.5f
        repeat(60) { animator.advance(dt) }
        assertEquals(1.0f, animator.facing, 0.02f)
    }

    @Test
    fun `a queued clip waits for the loop to wrap`() {
        // A stop clip must begin on the loop's first pose; starting it mid-stride would skate.
        val stop = List(10) { FloatArray(7 + JOINTS.size).also { f -> f[3] = 1f } }
        val animator = Animator(syntheticLibrary(period = 20, extra = mapOf("walk_stop" to stop)), FakeBody())
        animator.play("walk", 0f)
        animator.advance(0.1f)
        animator.queue("walk_stop")
        animator.advance(0.1f)
        assertTrue(animator.isPlaying("walk"))
        animator.advance(0.25f)
        assertTrue(animator.isPlaying("walk_stop"))
    }

    @Test
    fun `looking up tips the beak up and looking left turns it left`() {
        // The signs measured in MuJoCo from the standing pose: +neck_pitch tips the beak up,
        // +head_pitch tips it down, +head_yaw turns it left.
        val body = FakeBody()
        val animator = Animator(syntheticLibrary(), body)
        val neck = TRUNK + JOINTS.indexOf("neck_pitch")
        val head = TRUNK + JOINTS.indexOf("head_pitch")
        val yaw = TRUNK + JOINTS.indexOf("head_yaw")
        animator.lookAt(0.3f, 0f, 0.8f)
        repeat(60) { animator.advance(dt) }
        assertTrue("neck ${body.frame[neck]}", body.frame[neck] > 0.1f)
        assertTrue("head ${body.frame[head]}", body.frame[head] < -0.1f)
        animator.lookAt(0.3f, 0.6f, 0.12f)
        repeat(60) { animator.advance(dt) }
        assertTrue("yaw ${body.frame[yaw]}", body.frame[yaw] > 0.5f)
    }

    @Test
    fun `facing reads the trunk's heading`() {
        val animator = Animator(syntheticLibrary(), FakeBody())
        animator.place(0f, 0f, (PI / 2).toFloat())
        assertEquals((PI / 2).toFloat(), animator.facing, 1e-4f)
    }
}
