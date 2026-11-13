package io.github.samimreza.virtualduck

import io.github.samimreza.virtualduck.motion.Body
import io.github.samimreza.virtualduck.motion.MotionLibrary
import io.github.samimreza.virtualduck.pet.Voice
import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assume
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.cos
import kotlin.math.sin

/** Remembers the last pose it was given. */
class FakeBody : Body {
    var frame = FloatArray(0)
    var mouth = 0f

    override fun setPose(frame: FloatArray, mouth: Float) {
        this.frame = frame.copyOf()
        this.mouth = mouth
    }
}

class FakeVoice : Voice {
    val said = mutableListOf<String>()

    override fun say(tag: String) {
        said += tag
    }

    override fun mouth() = 0f

    override val speaking = false

    override fun sing() = 24f

    override fun hush() {
        said += "hush"
    }
}

/** The joints, in the order the recorder writes them. */
val JOINTS = listOf(
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)

/**
 * A library built in memory: a still "stand", a "walk" loop moving [stride] metres forward per
 * [period]-frame cycle, and whatever else [extra] adds — each a list of frames.
 */
fun syntheticLibrary(
    stride: Float = 0.05f,
    period: Int = 20,
    extra: Map<String, List<FloatArray>> = emptyMap(),
): MotionLibrary {
    val size = 7 + JOINTS.size
    fun frame(x: Float, yaw: Float = 0f, joint: Float = 0f) = FloatArray(size).also {
        it[0] = x
        it[2] = 0.116f
        it[3] = cos(yaw / 2)
        it[6] = sin(yaw / 2)
        for (j in 7 until size) it[j] = joint
    }
    val blocks = mutableListOf<FloatArray>()
    var offset = 0
    fun block(frames: List<FloatArray>): Int {
        val at = offset
        frames.forEach { blocks += it; offset += it.size }
        return at
    }
    val clips = JSONObject()
    fun clip(name: String, frames: List<FloatArray>, end: FloatArray? = null) {
        clips.put(
            name,
            JSONObject()
                .put("loop", end != null)
                .put("frames", frames.size)
                .put("offset", block(frames))
                .put("end", if (end != null) block(listOf(end)) else -1)
                .put("ball", -1),
        )
    }
    clip("stand", List(100) { frame(0f) })
    clip("walk", List(period) { frame(stride * it / period, joint = it.toFloat() / period) }, frame(stride, joint = 1f))
    for ((name, frames) in extra) clip(name, frames)
    val index = JSONObject()
        .put("source", JSONObject().put("rate_hz", 50))
        .put("joints", JSONArray(JOINTS))
        .put("frame_size", size)
        .put("clips", clips)
    val bytes = ByteBuffer.allocate(offset * 4).order(ByteOrder.LITTLE_ENDIAN)
    blocks.forEach { f -> f.forEach { bytes.putFloat(it) } }
    bytes.flip()
    return MotionLibrary.load(index.toString(), bytes)
}

/**
 * The motions `tools/build_assets.sh` recorded from the real policies. Generated, not committed, so
 * a checkout that has not built them skips the tests that need them rather than failing.
 */
fun recordedLibrary(): MotionLibrary {
    val dir = File("src/main/assets/duck")
    Assume.assumeTrue(
        "no recorded motions in ${dir.absolutePath}; run virtual-duck/tools/build_assets.sh",
        File(dir, "motions.bin").exists(),
    )
    return MotionLibrary.load(File(dir, "motions.json").readText(), ByteBuffer.wrap(File(dir, "motions.bin").readBytes()))
}
