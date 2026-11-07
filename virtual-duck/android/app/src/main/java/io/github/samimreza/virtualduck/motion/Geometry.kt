package io.github.samimreza.virtualduck.motion

import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

// The duck lives in the simulator's frame — X forward, Y left, Z up, metres — because that is the
// frame its motions were recorded in, and keeping it there means a recorded frame is applied with
// no arithmetic. Only the renderer converts, once, to Filament's Y-up.

data class Vec2(val x: Float, val y: Float) {
    operator fun plus(o: Vec2) = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2) = Vec2(x - o.x, y - o.y)
    fun length() = sqrt(x * x + y * y)
}

data class Vec3(val x: Float, val y: Float, val z: Float) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun times(s: Float) = Vec3(x * s, y * s, z * s)
    fun lerp(o: Vec3, t: Float) = Vec3(x + (o.x - x) * t, y + (o.y - y) * t, z + (o.z - z) * t)
    fun length() = sqrt(x * x + y * y + z * z)

    companion object {
        val ZERO = Vec3(0f, 0f, 0f)
    }
}

/** A rotation, (w, x, y, z) as MuJoCo writes it. */
data class Quat(val w: Float, val x: Float, val y: Float, val z: Float) {
    operator fun times(o: Quat) = Quat(
        w * o.w - x * o.x - y * o.y - z * o.z,
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w,
    )

    fun conjugate() = Quat(w, -x, -y, -z)

    fun rotate(v: Vec3): Vec3 {
        val p = this * Quat(0f, v.x, v.y, v.z) * conjugate()
        return Vec3(p.x, p.y, p.z)
    }

    fun normalized(): Quat {
        val n = sqrt(w * w + x * x + y * y + z * z)
        return Quat(w / n, x / n, y / n, z / n)
    }

    fun slerp(o: Quat, t: Float): Quat {
        var dot = w * o.w + x * o.x + y * o.y + z * o.z
        val b = if (dot < 0f) Quat(-o.w, -o.x, -o.y, -o.z).also { dot = -dot } else o
        if (dot > 0.9995f) {
            return Quat(w + (b.w - w) * t, x + (b.x - x) * t, y + (b.y - y) * t, z + (b.z - z) * t).normalized()
        }
        val theta = acos(dot)
        val s = sin(theta)
        val wa = sin((1 - t) * theta) / s
        val wb = sin(t * theta) / s
        return Quat(w * wa + b.w * wb, x * wa + b.x * wb, y * wa + b.y * wb, z * wa + b.z * wb)
    }

    /** The way it faces: the heading of its X axis about Z, radians. */
    fun heading(): Float {
        val forward = rotate(Vec3(1f, 0f, 0f))
        return atan2(forward.y, forward.x)
    }

    companion object {
        val IDENTITY = Quat(1f, 0f, 0f, 0f)

        fun axisAngle(axis: Vec3, angle: Float): Quat {
            val s = sin(angle / 2)
            return Quat(cos(angle / 2), axis.x * s, axis.y * s, axis.z * s)
        }

        fun yaw(angle: Float) = axisAngle(Vec3(0f, 0f, 1f), angle)
    }
}

/**
 * A place on the floor and the way it faces: what a clip is stored relative to, and what the
 * animator carries the duck through the room by. Composes like a 2D rigid transform.
 */
data class Ground(val x: Float, val y: Float, val yaw: Float) {
    operator fun times(o: Ground): Ground {
        val c = cos(yaw)
        val s = sin(yaw)
        return Ground(x + c * o.x - s * o.y, y + s * o.x + c * o.y, yaw + o.yaw)
    }

    fun apply(p: Vec2): Vec2 {
        val c = cos(yaw)
        val s = sin(yaw)
        return Vec2(x + c * p.x - s * p.y, y + s * p.x + c * p.y)
    }

    fun inverse(): Ground {
        val c = cos(yaw)
        val s = sin(yaw)
        return Ground(-(c * x + s * y), -(-s * x + c * y), -yaw)
    }

    /** This frame turned by `angle` about the floor point `pivot`. */
    fun turnedAbout(pivot: Vec2, angle: Float) = Ground(pivot.x, pivot.y, angle) * Ground(-pivot.x, -pivot.y, 0f) * this

    companion object {
        val ORIGIN = Ground(0f, 0f, 0f)
    }
}

/** `angle` folded into (-π, π]. */
fun wrapAngle(angle: Float): Float = atan2(sin(angle), cos(angle))
