package io.github.samimreza.virtualduck

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ApplicationInfo
import android.graphics.Bitmap
import android.os.Build
import java.io.File

/**
 * Screenshots of the duck for developers, in debuggable builds only:
 *
 *     adb shell am broadcast -a io.github.samimreza.virtualduck.CAPTURE
 *     adb shell run-as io.github.samimreza.virtualduck cat files/captures/overlay.png > overlay.png
 *
 * Each surface writes what Filament drew, read back from the renderer — the overlay with its
 * transparency intact — which is also the only way to see it on an emulator whose screen capture
 * cannot read GPU surfaces.
 */
class Capture(private val context: Context, private val name: String, private val stage: () -> DuckStage?) {
    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context, intent: Intent) {
            stage()?.renderer?.capture { bitmap ->
                val dir = File(context.filesDir, "captures").apply { mkdirs() }
                File(dir, "$name.png").outputStream().use { bitmap.compress(Bitmap.CompressFormat.PNG, 100, it) }
            }
        }
    }

    fun register() {
        if (context.applicationInfo.flags and ApplicationInfo.FLAG_DEBUGGABLE == 0) return
        val filter = IntentFilter(ACTION)
        if (Build.VERSION.SDK_INT >= 33) {
            context.registerReceiver(receiver, filter, Context.RECEIVER_EXPORTED)
        } else {
            context.registerReceiver(receiver, filter)
        }
        registered = true
    }

    fun unregister() {
        if (registered) runCatching { context.unregisterReceiver(receiver) }
        registered = false
    }

    private var registered = false

    companion object {
        const val ACTION = "io.github.samimreza.virtualduck.CAPTURE"
    }
}
