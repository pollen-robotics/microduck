package io.github.samimreza.virtualduck

import android.content.Context
import io.github.samimreza.virtualduck.motion.MotionLibrary
import io.github.samimreza.virtualduck.motion.Rig
import io.github.samimreza.virtualduck.pet.PetState
import org.json.JSONObject
import java.nio.ByteBuffer
import java.util.concurrent.Executors

/**
 * Everything `tools/build_assets.sh` put in assets/duck, read once per process and shared by the
 * overlay and the home screen. Reading is a few tens of milliseconds and happens off the main
 * thread; the renderers built from it are each their own.
 */
class DuckAssets private constructor(
    val glb: ByteArray,
    val rig: Rig,
    val motions: MotionLibrary,
    val voices: JSONObject,
) {
    /** A direct buffer of the model, which is what Filament reads. Filament copies it; one per renderer. */
    fun glbBuffer(): ByteBuffer = ByteBuffer.allocateDirect(glb.size).put(glb).also { it.flip() }

    /**
     * The voice styles `tools/render_voices.py` rendered, the default first: id and name. None if the
     * index is from an older renderer — then the duck is silent rather than the app failing to start.
     */
    val voiceStyles: List<Pair<String, String>>
        get() = voices.optJSONArray("styles")?.let { a ->
            List(a.length()) { a.getJSONObject(it).let { style -> style.getString("id") to style.getString("name") } }
        }.orEmpty()

    companion object {
        @Volatile
        private var loaded: DuckAssets? = null
        private val io = Executors.newSingleThreadExecutor()

        /** Calls [ready] on the main thread once the assets are loaded — at once, if they already are. */
        fun get(context: Context, ready: (DuckAssets) -> Unit) {
            loaded?.let { return ready(it) }
            val app = context.applicationContext
            val main = android.os.Handler(android.os.Looper.getMainLooper())
            io.execute {
                val assets = loaded ?: read(app).also { loaded = it }
                main.post { ready(assets) }
            }
        }

        private fun read(context: Context): DuckAssets {
            val a = context.assets
            fun text(path: String) = a.open("duck/$path").bufferedReader().use { it.readText() }
            fun bytes(path: String) = a.open("duck/$path").use { it.readBytes() }
            val bin = bytes("motions.bin")
            return DuckAssets(
                bytes("duck.glb"),
                Rig.parse(text("duck_rig.json")),
                MotionLibrary.load(text("motions.json"), ByteBuffer.wrap(bin)),
                JSONObject(text("voices.json")),
            )
        }
    }
}

/**
 * The one duck, shared by the overlay and the home screen, and where it is kept between runs. They
 * never animate it at once: the overlay steps aside while home is showing.
 */
object PetHub {
    private const val PREFS = "duck"
    private const val KEY = "state"

    private var state: PetState? = null

    /** Home is in front: the overlay hides and stops drawing until it is not. */
    var homeVisible = false
        set(value) {
            field = value
            listeners.forEach { it() }
        }

    val listeners = mutableListOf<() -> Unit>()

    /** Feeds the duck on the home screen, while it is in front: where the notification's Feed goes. */
    var feedHome: (() -> Unit)? = null

    fun state(context: Context): PetState {
        state?.let { return it }
        val prefs = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
        val now = System.currentTimeMillis()
        val pet = prefs.getString(KEY, null)?.let { PetState.fromJson(it) } ?: PetState.hatch(now)
        pet.catchUp(now)
        state = pet
        return pet
    }

    fun save(context: Context) {
        val pet = state ?: return
        write(context, pet)
    }

    /** The duck in memory if there is one, else as last saved — for the reminder, with no screen up. */
    fun load(context: Context): PetState? = state
        ?: context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).getString(KEY, null)?.let { PetState.fromJson(it) }

    fun write(context: Context, pet: PetState) {
        pet.updatedAtMs = System.currentTimeMillis()
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().putString(KEY, pet.toJson()).apply()
    }
}
