package io.github.samimreza.virtualduck

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ApplicationInfo
import android.graphics.Bitmap
import android.os.Build
import org.json.JSONObject
import java.io.File

/**
 * Drives a duck from adb, in debuggable builds only — what `tools/e2e.sh` tests the real app with,
 * on a device or emulator, over whatever app is in front:
 *
 *     adb shell am broadcast -a io.github.samimreza.virtualduck.DEBUG -p io.github.samimreza.virtualduck \
 *         --es target overlay --es cmd feed
 *     adb shell run-as io.github.samimreza.virtualduck cat files/debug/overlay.json
 *
 * Commands: feed, ball, dance, trick, sing, sleep, poke, stroke, listen (a sound starts and never becomes
 * speech), heard:<syllables>, shake, come:<metres>, voice:<style>, pitch:<0-2>, speed:<times>, capture,
 * status. Every command rewrites the target's status file afterwards, so a script can poll it. A release
 * build registers nothing.
 */
class DebugBridge(
    private val context: Context,
    private val name: String,
    private val stage: () -> DuckStage?,
) {
    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context, intent: Intent) {
            if (intent.getStringExtra("target") != name) return
            val duck = stage() ?: return
            val brain = duck.brain
            val cmd = intent.getStringExtra("cmd") ?: "status"
            val arg = cmd.substringAfter(':', "")
            when (cmd.substringBefore(':')) {
                "feed" -> brain.feed()
                "ball" -> brain.throwBall()
                "dance" -> brain.dance()
                "trick" -> brain.trick()
                "sing" -> brain.sing()
                "sleep" -> brain.toggleSleep()
                "poke" -> brain.poke(duck.animator.floorX, 0.2f)
                "stroke" -> brain.stroke(duck.animator.floorX, 0.25f)
                "listen" -> brain.listen()
                "heard" -> brain.heard(arg.toIntOrNull() ?: 3, listOf(0.2f, 0.25f, 0.3f))
                "shake" -> brain.shaken(1f)
                "come" -> arg.toFloatOrNull()?.let { brain.come(it) }
                "voice", "pitch" -> {
                    if (cmd.startsWith("voice")) brain.state.voice = arg else brain.state.voicePitch = arg.toIntOrNull()?.coerceIn(0, 2) ?: 1
                    duck.voice.reload()
                    duck.voice.say("greet")
                }
                "speed" -> duck.timeScale = arg.toFloatOrNull()?.coerceIn(0.1f, 20f) ?: 1f
                "capture" -> duck.renderer.capture { bitmap ->
                    file("$name.png").outputStream().use { bitmap.compress(Bitmap.CompressFormat.PNG, 100, it) }
                }
            }
            writeStatus()
        }
    }

    /** The duck's state as JSON, for a script to read with run-as. */
    fun writeStatus() {
        val duck = stage() ?: return
        val status = JSONObject(duck.brain.status().mapValues { it.value ?: JSONObject.NULL })
            .put("where", name)
            .put("time_scale", duck.timeScale.toDouble())
        file("$name.json").writeText(status.toString())
    }

    private fun file(child: String) = File(File(context.filesDir, "debug").apply { mkdirs() }, child)

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
        const val ACTION = "io.github.samimreza.virtualduck.DEBUG"
    }
}
