package io.github.samimreza.virtualduck.pet

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import kotlin.math.sqrt

/**
 * Notices the phone being shaken: [JOLTS] jolts of more than [JOLT] m/s² beyond gravity within
 * [WINDOW] seconds, then quiet for [COOLDOWN]. A walk, a bus, a phone put down on a table are one
 * jolt or a gentle sway; a shake is several hard ones close together. Pure, so it can be tested.
 */
class ShakeDetector(private val onShake: (direction: Float) -> Unit) {
    private val jolts = ArrayDeque<Float>()
    private var quietUntil = 0f
    private var sideways = 0f

    /** One accelerometer sample at [t] seconds: x across the screen, y up it, z out of it. */
    fun sample(t: Float, x: Float, y: Float, z: Float) {
        val beyond = sqrt(x * x + y * y + z * z) - SensorManager.GRAVITY_EARTH
        if (t < quietUntil || beyond < JOLT) return
        if (jolts.isNotEmpty() && t - jolts.last() < 0.08f) return
        jolts.addLast(t)
        sideways += x
        while (jolts.isNotEmpty() && t - jolts.first() > WINDOW) jolts.removeFirst()
        if (jolts.size >= JOLTS) {
            jolts.clear()
            quietUntil = t + COOLDOWN
            onShake(sideways)
            sideways = 0f
        }
    }

    companion object {
        const val JOLT = 11f
        const val JOLTS = 3
        const val WINDOW = 0.9f
        const val COOLDOWN = 3f
    }
}

/** The accelerometer, at the rate meant for UI — cheap, and plenty to feel a shake. */
class ShakeSensor(context: Context, onShake: (Float) -> Unit) : SensorEventListener {
    private val sensors = context.getSystemService(SensorManager::class.java)
    private val accelerometer = sensors?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val detector = ShakeDetector(onShake)
    private var listening = false

    fun start() {
        if (listening || accelerometer == null) return
        listening = sensors.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI)
    }

    fun stop() {
        if (!listening) return
        sensors.unregisterListener(this)
        listening = false
    }

    override fun onSensorChanged(event: SensorEvent) {
        detector.sample(event.timestamp / 1e9f, event.values[0], event.values[1], event.values[2])
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
