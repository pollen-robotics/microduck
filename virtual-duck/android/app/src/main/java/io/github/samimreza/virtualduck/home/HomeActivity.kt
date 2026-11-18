package io.github.samimreza.virtualduck.home

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.content.Intent
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.RippleDrawable
import android.graphics.drawable.StateListDrawable
import android.media.AudioManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.provider.Settings
import android.text.Editable
import android.text.TextWatcher
import android.view.Gravity
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.SurfaceView
import android.view.View
import android.view.ViewGroup
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import io.github.samimreza.virtualduck.Capture
import io.github.samimreza.virtualduck.DebugBridge
import io.github.samimreza.virtualduck.DuckAssets
import io.github.samimreza.virtualduck.DuckStage
import io.github.samimreza.virtualduck.PetHub
import io.github.samimreza.virtualduck.R
import io.github.samimreza.virtualduck.Reminders
import io.github.samimreza.virtualduck.overlay.BallView
import io.github.samimreza.virtualduck.overlay.FloorView
import io.github.samimreza.virtualduck.overlay.PetService
import io.github.samimreza.virtualduck.overlay.SeedsView
import io.github.samimreza.virtualduck.pet.Ball
import io.github.samimreza.virtualduck.pet.Listener
import io.github.samimreza.virtualduck.pet.Mood
import io.github.samimreza.virtualduck.pet.PetState
import io.github.samimreza.virtualduck.pet.ShakeSensor
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.roundToInt

/**
 * The duck's home: where it is when it is not out on the screen, and where its owner sees how it
 * is and how far the two of them have come — friendship level, streak, the things done together —
 * and names it, picks its voice, its size and where it walks, and lets it out. Plain framework
 * views built in code — no AppCompat, no Compose — so the APK is the duck, Filament, and very
 * little else.
 *
 * The room is the same duck with the same brain as on the overlay, at the size it will be out
 * there: tap it, stroke it, talk to it, shake the phone, or tap the floor to call it over.
 */
class HomeActivity : Activity() {
    private var stage: DuckStage? = null
    private lateinit var state: PetState
    private lateinit var assets: DuckAssets
    private lateinit var surface: SurfaceView
    private lateinit var floor: FloorView
    private lateinit var seeds: SeedsView
    private lateinit var ballView: BallView
    private lateinit var moodText: TextView
    private lateinit var levelText: TextView
    private lateinit var streakText: TextView
    private lateinit var statsText: TextView
    private lateinit var levelBar: ProgressBar
    private lateinit var outButton: Button
    private lateinit var singButton: Button
    private lateinit var talkButton: Button
    private val voiceButtons = mutableListOf<Button>()
    private val pitchButtons = mutableListOf<Button>()
    private val bars = mutableMapOf<String, ProgressBar>()
    private val main = Handler(Looper.getMainLooper())
    private val point = FloatArray(3)
    private val screen = FloatArray(2)
    private var pixelsPerMeter = 700f
    private var targetPixelsPerMeter = 0f
    private var resumed = false
    private var seedsVersion = -1
    private var askedForOverlay = false
    private val density get() = resources.displayMetrics.density
    private val capture = Capture(this, "home") { stage }

    private val debug = DebugBridge(this, "home") { stage }

    private val ears by lazy {
        Listener(
            this,
            onStart = { stage?.brain?.listen() },
            onEnd = { heard -> stage?.brain?.heard(heard.syllables, heard.gaps) },
            onCancel = { stage?.brain?.stopListening() },
        )
    }
    private val shake by lazy { ShakeSensor(this) { sideways -> stage?.brain?.shaken(sideways) } }

    private val refresh = object : Runnable {
        override fun run() {
            updateNeeds()
            main.postDelayed(this, 1000L)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // The volume keys turn the duck up or down while its home is open.
        volumeControlStream = AudioManager.STREAM_MUSIC
        capture.register()
        debug.register()
        Reminders.schedule(this)
        setContentView(TextView(this).apply {
            text = getString(R.string.loading)
            gravity = Gravity.CENTER
            setBackgroundColor(BACKGROUND)
        })
        DuckAssets.get(this) { loaded ->
            assets = loaded
            state = PetHub.state(this)
            build()
        }
    }

    override fun onResume() {
        super.onResume()
        resumed = true
        PetHub.homeVisible = true
        PetHub.feedHome = { stage?.brain?.feed() }
        // The overlay's duck shares its state, and may have slept or woken meanwhile.
        stage?.brain?.sync()
        stage?.start()
        main.post(refresh)
        if (this::outButton.isInitialized) updateOutButton()
        senses(true)
        if (askedForOverlay) {
            // Back from the permission screen: out it goes if it was granted; if not, say why.
            askedForOverlay = false
            if (Settings.canDrawOverlays(this)) letOut() else explainBlocked()
        }
    }

    override fun onPause() {
        super.onPause()
        resumed = false
        senses(false)
        stage?.stop()
        main.removeCallbacks(refresh)
        PetHub.feedHome = null
        PetHub.homeVisible = false
        PetHub.save(this)
    }

    override fun onDestroy() {
        capture.unregister()
        debug.unregister()
        stage?.destroy()
        stage = null
        super.onDestroy()
    }

    /** Ears and the shake sensor, only while home is in front — and ears only if asked for. */
    private fun senses(on: Boolean) {
        if (!this::state.isInitialized) return
        if (on && stage != null) {
            shake.start()
            if (state.talk) ears.start() else ears.stop()
        } else {
            shake.stop()
            ears.stop()
        }
    }

    // ---------------------------------------------------------------- building the screen

    private fun build() {
        val column = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(BACKGROUND)
        }

        // Tall enough for a Large duck and the thought over its head.
        val roomHeight = (320 * density).roundToInt()
        // The duck at home is the size it will be out on the screen, so the room previews the size
        // setting rather than ignoring it.
        pixelsPerMeter = scaleFor(state.sizeDp)
        targetPixelsPerMeter = pixelsPerMeter
        val room = FrameLayout(this)
        surface = SurfaceView(this)
        floor = FloorView(this)
        seeds = SeedsView(this)
        ballView = BallView(this)
        room.addView(surface, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, roomHeight))
        room.addView(floor, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, roomHeight))
        room.addView(seeds, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, roomHeight))
        val ballSize = (Ball.RADIUS * 2.2f * pixelsPerMeter).roundToInt()
        room.addView(ballView, FrameLayout.LayoutParams(ballSize, ballSize))
        column.addView(room, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, roomHeight))

        val body = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            val pad = (20 * density).roundToInt()
            setPadding(pad, pad / 2, pad, pad)
        }

        // Who it is, and how far the two of you have come.
        val header = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        header.addView(nameField(), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        levelText = TextView(this).apply {
            textSize = 14f
            typeface = Typeface.DEFAULT_BOLD
            setTextColor(Color.WHITE)
            val pad = (10 * density).roundToInt()
            setPadding(pad, pad / 2, pad, pad / 2)
            background = GradientDrawable().apply {
                cornerRadius = 14 * density
                setColor(ACCENT)
            }
        }
        header.addView(levelText)
        body.addView(header)
        levelBar = ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal).apply {
            progressTintList = ColorStateList.valueOf(ACCENT)
        }
        body.addView(levelBar, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, (8 * density).roundToInt()))
        streakText = label("", 14f, bold = true)
        body.addView(streakText)
        moodText = label("", 15f, bold = false)
        body.addView(moodText)
        for ((key, title) in listOf("food" to R.string.need_food, "energy" to R.string.need_energy, "joy" to R.string.need_joy, "love" to R.string.need_love)) {
            body.addView(needRow(key, getString(title)))
        }

        body.addView(row(
            button("🌾 " + getString(R.string.feed)) { stage?.brain?.feed() },
            button("⚽ " + getString(R.string.ball)) { stage?.brain?.throwBall() },
            button("💃 " + getString(R.string.dance)) { stage?.brain?.dance() },
        ))
        singButton = button("") { sing() }
        body.addView(row(
            singButton,
            button("🤸 " + getString(R.string.trick)) { stage?.brain?.trick() },
            button("💤 " + getString(R.string.sleep)) { stage?.brain?.toggleSleep() },
        ))
        body.addView(label(getString(R.string.room_hint), 13f, bold = false).apply { setTextColor(MUTED) })

        outButton = button("", primary = true) { toggleOverlay() }
        body.addView(outButton, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, (52 * density).roundToInt()).apply {
            topMargin = (16 * density).roundToInt()
        })
        body.addView(label(getString(R.string.out_explained), 13f, bold = false))

        // Talking to it.
        body.addView(label(getString(R.string.talk), 16f, bold = true))
        talkButton = button("") { toggleTalk() }
        showTalk()
        body.addView(talkButton, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, (48 * density).roundToInt()))
        body.addView(label(getString(R.string.talk_explained), 13f, bold = false).apply { setTextColor(MUTED) })

        // Where on the screen it lives.
        body.addView(label(getString(R.string.where), 16f, bold = true))
        val levels = listOf(1f, 0.5f, 0f)
        val whereButtons = mutableListOf<Button>()
        for ((i, name) in listOf(R.string.where_bottom, R.string.where_middle, R.string.where_top).withIndex()) {
            whereButtons += button(getString(name)) {
                state.floorLevel = levels[i]
                choose(whereButtons, i)
                refreshOverlay()
            }
        }
        choose(whereButtons, levels.indexOfFirst { abs(it - state.floorLevel) < 0.01f })
        body.addView(row(*whereButtons.toTypedArray()))
        body.addView(label(getString(R.string.where_note), 13f, bold = false).apply { setTextColor(MUTED) })

        // Voice, pitch and size are choices: the chosen one stays lit, so a press is seen to have taken.
        body.addView(label(getString(R.string.voice), 16f, bold = true))
        for ((id, name) in assets.voiceStyles) voiceButtons += button(name) { pickVoice(id) }
        body.addView(row(*voiceButtons.toTypedArray()))
        for ((i, name) in listOf(R.string.pitch_low, R.string.pitch_middle, R.string.pitch_high).withIndex()) {
            pitchButtons += button(getString(name)) { pickPitch(i) }
        }
        body.addView(row(*pitchButtons.toTypedArray()))
        body.addView(label(getString(R.string.voice_note), 13f, bold = false).apply { setTextColor(MUTED) })
        showVoices()

        body.addView(label(getString(R.string.size), 16f, bold = true))
        val sizes = listOf(110, 150, 200)
        val sizeButtons = mutableListOf<Button>()
        for ((i, name) in listOf(R.string.size_small, R.string.size_medium, R.string.size_large).withIndex()) {
            sizeButtons += button(getString(name)) {
                state.sizeDp = sizes[i]
                choose(sizeButtons, i)
                // The room's duck grows or shrinks to it.
                targetPixelsPerMeter = scaleFor(sizes[i])
            }
        }
        choose(sizeButtons, sizes.indexOf(state.sizeDp).coerceAtLeast(1))
        body.addView(row(*sizeButtons.toTypedArray()))
        body.addView(label(getString(R.string.size_note), 13f, bold = false).apply { setTextColor(MUTED) })

        body.addView(label(getString(R.string.sound), 16f, bold = true))
        val soundButtons = mutableListOf<Button>()
        for ((i, name) in listOf(R.string.sound_off, R.string.sound_soft, R.string.sound_normal).withIndex()) {
            soundButtons += button(getString(name)) {
                state.soundLevel = i
                choose(soundButtons, i)
            }
        }
        choose(soundButtons, state.soundLevel.coerceIn(0, 2))
        body.addView(row(*soundButtons.toTypedArray()))

        statsText = label("", 13f, bold = false).apply { setTextColor(MUTED) }
        body.addView(statsText)
        body.addView(label(getString(R.string.credits), 12f, bold = false).apply { setTextColor(MUTED) })
        column.addView(body)
        val scroll = ScrollView(this).apply { addView(column) }
        // Android 15 draws every app edge to edge: keep the page out from under the system bars.
        scroll.setOnApplyWindowInsetsListener { v, insets ->
            if (Build.VERSION.SDK_INT >= 30) {
                val systemBars = insets.getInsets(WindowInsets.Type.systemBars())
                v.setPadding(0, systemBars.top, 0, systemBars.bottom)
            } else {
                @Suppress("DEPRECATION")
                v.setPadding(0, insets.systemWindowInsetTop, 0, insets.systemWindowInsetBottom)
            }
            insets
        }
        setContentView(scroll)
        if (Build.VERSION.SDK_INT >= 30) {
            val light = WindowInsetsController.APPEARANCE_LIGHT_STATUS_BARS or WindowInsetsController.APPEARANCE_LIGHT_NAVIGATION_BARS
            window.insetsController?.setSystemBarsAppearance(light, light)
        }
        surface.setOnTouchListener(RoomTouch())
        updateNeeds()
        updateOutButton()

        // The duck after the page's first frame: building it holds the main thread for a moment,
        // and the page should already be on screen when it does.
        surface.post {
            if (isFinishing || isDestroyed) return@post
            val duck = DuckStage(this, surface, transparent = false, assets, state)
            duck.renderer.pixelsPerMeter = pixelsPerMeter
            duck.beforeRender = ::follow
            duck.brain.left = 0.12f
            duck.brain.right = surface.width / pixelsPerMeter - 0.12f
            duck.animator.place(surface.width / pixelsPerMeter / 2, 0f, duck.animator.facing)
            duck.brain.onLevelUp = ::celebrate
            stage = duck
            if (resumed) {
                duck.start()
                senses(true)
            }
        }
    }

    private fun scaleFor(sizeDp: Int) = sizeDp * density / DUCK_TALL

    /** The room's camera stays put; the duck walks across it. */
    private fun follow() {
        val duck = stage ?: return
        if (abs(targetPixelsPerMeter - pixelsPerMeter) > 0.5f) resize(duck)
        ears.deaf = duck.voice.speaking
        val width = surface.width / pixelsPerMeter
        duck.renderer.frame(width / 2, FLOOR_FROM_BOTTOM)
        val brain = duck.brain
        duck.renderer.project(duck.animator.floorX, 0f, 0f, screen)
        // Redraw the shadow and bubble only when something moved, not every frame.
        var moved = abs(floor.shadowX - screen[0]) >= 1f || floor.bubble != brain.bubble || brain.bubble != null
        floor.shadowX = screen[0]
        floor.shadowY = screen[1]
        floor.shadowWidth = 0.17f * pixelsPerMeter
        floor.lifted = (brain.lift / 0.08f).coerceIn(0f, 1f)
        floor.bubble = brain.bubble
        if (brain.bubble != null || brain.hearts > 0) {
            duck.renderer.headPosition(point)
            duck.renderer.project(point[0], point[1], point[2] + 0.09f, screen)
            floor.bubbleX = screen[0]
            floor.bubbleY = screen[1]
            if (brain.hearts > 0) {
                floor.burst(brain.hearts, screen[0], screen[1] + floor.shadowWidth * 0.3f)
                brain.hearts = 0
            }
        }
        if (floor.stepHearts()) moved = true
        if (moved) floor.invalidate()
        if (seedsVersion != brain.pelletsVersion) {
            seedsVersion = brain.pelletsVersion
            seeds.seedWidth = 0.022f * pixelsPerMeter
            seeds.seeds = FloatArray(brain.pellets.size) {
                duck.renderer.project(brain.pellets[it], 0f, 0f, screen)
                screen[0]
            }
        }
        val ball = brain.ball
        ballView.visibility = if (ball == null) View.GONE else View.VISIBLE
        if (ball != null) {
            duck.renderer.project(ball.x, 0f, ball.z, screen)
            ballView.translationX = screen[0] - ballView.width / 2f
            ballView.translationY = screen[1] - ballView.height / 2f
            ballView.angle = ball.angle
        }
    }

    /**
     * One step of growing or shrinking toward the chosen size — a third of the way each frame, so a
     * change takes a few tenths of a second and is seen to happen. The duck keeps its place on the
     * screen while it does: its floor position is rescaled with the room.
     */
    private fun resize(duck: DuckStage) {
        val step = pixelsPerMeter + (targetPixelsPerMeter - pixelsPerMeter) * 0.35f
        val next = if (abs(targetPixelsPerMeter - step) <= 0.5f) targetPixelsPerMeter else step
        val factor = pixelsPerMeter / next
        pixelsPerMeter = next
        duck.renderer.pixelsPerMeter = pixelsPerMeter
        duck.animator.place(duck.animator.floorX * factor, duck.animator.floorY, duck.animator.facing)
        duck.brain.rescale(factor)
        duck.brain.right = surface.width / pixelsPerMeter - 0.12f
        val ballSize = (Ball.RADIUS * 2.2f * pixelsPerMeter).roundToInt()
        ballView.layoutParams = FrameLayout.LayoutParams(ballSize, ballSize)
        seedsVersion = -1
    }

    private fun updateNeeds() {
        if (!this::moodText.isInitialized) return
        bars["food"]?.progress = (state.food * 100).roundToInt()
        bars["energy"]?.progress = (state.energy * 100).roundToInt()
        bars["joy"]?.progress = (state.joy * 100).roundToInt()
        bars["love"]?.progress = (state.love * 100).roundToInt()
        moodText.text = getString(
            when (state.mood) {
                Mood.HAPPY -> R.string.mood_happy
                Mood.CONTENT -> R.string.mood_content
                Mood.HUNGRY -> R.string.mood_hungry
                Mood.SLEEPY -> R.string.mood_sleepy
                Mood.LONELY -> R.string.mood_lonely
                Mood.BORED -> R.string.mood_bored
            },
        )
        levelText.text = getString(R.string.level, state.level)
        val (done, needed) = state.levelProgress
        levelBar.max = needed
        levelBar.progress = done
        val days = ((System.currentTimeMillis() - state.bornAtMs) / 86_400_000L + 1).coerceAtLeast(1)
        streakText.text = if (state.streak > 0) {
            getString(R.string.streak, state.streak, days)
        } else {
            getString(R.string.together, days)
        }
        statsText.text = getString(R.string.stats, state.timesPetted, state.pelletsEaten, state.kicks, state.chats)
        singButton.text = if (state.canSing) "🎵 " + getString(R.string.sing) else getString(R.string.sing_locked)
    }

    private fun celebrate(level: Int) {
        val unlocked = if (level == PetState.SING_LEVEL) getString(R.string.unlocked_song) else ""
        Toast.makeText(this, getString(R.string.level_up, level) + " " + unlocked, Toast.LENGTH_LONG).show()
        updateNeeds()
    }

    private fun sing() {
        if (!state.canSing) {
            Toast.makeText(this, getString(R.string.sing_unlocks, PetState.SING_LEVEL), Toast.LENGTH_SHORT).show()
            return
        }
        stage?.brain?.sing()
    }

    private fun pickVoice(id: String) {
        state.voice = id
        voiceChanged()
    }

    private fun pickPitch(index: Int) {
        state.voicePitch = index
        voiceChanged()
    }

    private fun voiceChanged() {
        showVoices()
        stage?.voice?.reload()
        // Waits for the new voice to load, then greets in it: the one sound a button makes, because
        // hearing the voice is the point of choosing one.
        stage?.voice?.say("greet")
    }

    private fun showVoices() {
        val styles = assets.voiceStyles.map { it.first }
        choose(voiceButtons, styles.indexOf(state.voice).takeIf { it >= 0 } ?: 0)
        choose(pitchButtons, state.voicePitch.coerceIn(0, pitchButtons.size - 1))
    }

    private fun toggleTalk() {
        if (!state.talk && checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(arrayOf(Manifest.permission.RECORD_AUDIO), REQUEST_MIC)
            return
        }
        state.talk = !state.talk
        showTalk()
        senses(resumed)
        refreshOverlay()
    }

    @Deprecated("Framework callback")
    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        if (requestCode == REQUEST_MIC && grantResults.firstOrNull() == PackageManager.PERMISSION_GRANTED) {
            state.talk = true
            showTalk()
            senses(resumed)
            refreshOverlay()
        }
    }

    private fun showTalk() {
        talkButton.text = getString(if (state.talk) R.string.talk_on else R.string.talk_off)
        style(talkButton, lit = state.talk)
    }

    /** Tells the overlay — if it is out — that a setting it acts on changed. */
    private fun refreshOverlay() {
        if (PetService.running) startService(Intent(this, PetService::class.java).setAction(PetService.ACTION_REFRESH))
    }

    // ---------------------------------------------------------------- letting it out

    private fun toggleOverlay() {
        if (PetService.running) {
            PetService.stop(this)
            main.postDelayed(::updateOutButton, 200L)
            return
        }
        if (!Settings.canDrawOverlays(this)) {
            askedForOverlay = true
            startActivity(Intent(Settings.ACTION_MANAGE_OVERLAY_PERMISSION, Uri.parse("package:$packageName")))
            return
        }
        letOut()
    }

    private fun letOut() {
        if (Build.VERSION.SDK_INT >= 33 && checkSelfPermission(Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED) {
            // Optional: without it the duck still comes out, its notification is just not shown.
            requestPermissions(arrayOf(Manifest.permission.POST_NOTIFICATIONS), REQUEST_NOTIFICATIONS)
        }
        PetHub.save(this)
        PetService.start(this)
        // Out it goes — and home steps back so it can be seen.
        moveTaskToBack(true)
    }

    /**
     * Came back from Settings without the permission. From Android 13, an app installed from a file
     * rather than a store has some settings locked — and from Android 15 drawing over other apps is
     * one of them: the switch answers "App was denied access" until the app's own info page has
     * "Allow restricted settings" turned on. Nobody finds that unaided, so this says where it is.
     */
    private fun explainBlocked() {
        val restricted = Build.VERSION.SDK_INT >= 33
        AlertDialog.Builder(this)
            .setTitle(if (restricted) R.string.blocked_title else R.string.allow_title)
            .setMessage(if (restricted) R.string.blocked_message else R.string.allow_message)
            .setPositiveButton(if (restricted) R.string.open_app_info else R.string.try_again) { _, _ ->
                if (restricted) {
                    startActivity(Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.parse("package:$packageName")))
                } else {
                    toggleOverlay()
                }
            }
            .setNegativeButton(R.string.not_now, null)
            .show()
    }

    private fun updateOutButton() {
        outButton.text = getString(
            when {
                PetService.running -> R.string.bring_home
                Settings.canDrawOverlays(this) -> R.string.let_out
                else -> R.string.allow_overlay
            },
        )
    }

    // ---------------------------------------------------------------- touch in the room

    /** Tap it: a poke. Double tap: a dance. Rub back and forth: petting. Tap the floor: come here. */
    private inner class RoomTouch : View.OnTouchListener {
        private var downX = 0f
        private var downY = 0f
        private var lastX = 0f
        private var lastY = 0f
        private var downTime = 0L
        private var path = 0f
        private var stroked = 0f
        private var lastTap = 0L

        @SuppressLint("ClickableViewAccessibility")
        override fun onTouch(v: View, event: MotionEvent): Boolean {
            val duck = stage ?: return false
            val slop = 12 * density
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    downX = event.x
                    downY = event.y
                    lastX = downX
                    lastY = downY
                    downTime = event.eventTime
                    path = 0f
                    stroked = 0f
                }
                MotionEvent.ACTION_MOVE -> {
                    val step = hypot(event.x - lastX, event.y - lastY)
                    path += step
                    if (path > 2.2f * hypot(event.x - downX, event.y - downY) && path > 5 * slop) {
                        stroked += step
                        if (stroked > 6 * slop) {
                            stroked = 0f
                            duck.renderer.unproject(event.x, event.y, point)
                            duck.brain.stroke(point[0], point[2])
                        }
                    }
                    lastX = event.x
                    lastY = event.y
                }
                MotionEvent.ACTION_UP -> if (event.eventTime - downTime < 350L && path < slop) {
                    duck.renderer.unproject(event.x, event.y, point)
                    if (abs(point[0] - duck.animator.floorX) > 0.12f) {
                        // Not on the duck: it comes to where the floor was tapped.
                        duck.brain.come(point[0])
                        return true
                    }
                    if (event.eventTime - lastTap < 320L) {
                        duck.brain.dance()
                        lastTap = 0L
                    } else {
                        duck.brain.poke(point[0], point[2])
                        lastTap = event.eventTime
                    }
                }
            }
            return true
        }
    }

    // ---------------------------------------------------------------- small views

    private fun nameField() = EditText(this).apply {
        setText(state.name)
        textSize = 26f
        typeface = Typeface.DEFAULT_BOLD
        setTextColor(INK)
        background = null
        setSingleLine()
        addTextChangedListener(object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) = Unit
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) = Unit
            override fun afterTextChanged(s: Editable?) {
                state.name = s?.toString()?.trim()?.ifEmpty { null } ?: "Duck"
            }
        })
    }

    private fun needRow(key: String, title: String): View {
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        row.addView(label(title, 14f, bold = false), LinearLayout.LayoutParams((80 * density).roundToInt(), ViewGroup.LayoutParams.WRAP_CONTENT))
        val bar = ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal).apply { max = 100 }
        bars[key] = bar
        row.addView(bar, LinearLayout.LayoutParams(0, (18 * density).roundToInt(), 1f))
        return row
    }

    private fun label(text: String, size: Float, bold: Boolean) = TextView(this).apply {
        this.text = text
        textSize = size
        setTextColor(INK)
        if (bold) typeface = Typeface.DEFAULT_BOLD
        val pad = (6 * density).roundToInt()
        setPadding(0, pad, 0, pad)
    }

    /**
     * A rounded button that shows it was pressed: a ripple and a darker fill under the finger, and
     * a tick of haptic feedback when the press lands — and no click sound: the feedback is the
     * duck doing what was asked.
     */
    private fun button(text: String, primary: Boolean = false, onClick: () -> Unit) = Button(this).apply {
        this.text = text
        isAllCaps = false
        textSize = 15f
        isSoundEffectsEnabled = false
        stateListAnimator = null
        style(this, lit = primary)
        setOnClickListener {
            performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            onClick()
        }
    }

    private fun style(button: Button, lit: Boolean) {
        val radius = 22 * density
        val fill = GradientDrawable().apply {
            cornerRadius = radius
            setColor(if (lit) ACCENT else Color.WHITE)
            setStroke((1 * density).roundToInt(), if (lit) ACCENT else 0x22000000)
        }
        val pressed = GradientDrawable().apply {
            cornerRadius = radius
            setColor(if (lit) ACCENT_DARK else PRESSED)
        }
        val states = StateListDrawable().apply {
            addState(intArrayOf(android.R.attr.state_pressed), pressed)
            addState(intArrayOf(), fill)
        }
        val mask = GradientDrawable().apply {
            cornerRadius = radius
            setColor(Color.WHITE)
        }
        button.background = RippleDrawable(ColorStateList.valueOf(if (lit) 0x55FFFFFF else 0x33FF671F), states, mask)
        button.setTextColor(if (lit) Color.WHITE else INK)
    }

    /** Lights the [index]th of a group of choices and unlights the rest. */
    private fun choose(group: List<Button>, index: Int) {
        group.forEachIndexed { i, b -> style(b, lit = i == index) }
    }

    private fun row(vararg views: View) = LinearLayout(this).apply {
        orientation = LinearLayout.HORIZONTAL
        for (v in views) {
            addView(v, LinearLayout.LayoutParams(0, (48 * density).roundToInt(), 1f).apply {
                val m = (4 * density).roundToInt()
                setMargins(m, m, m, m)
            })
        }
    }

    companion object {
        private val BACKGROUND = 0xFFF4EFE6.toInt()
        private val INK = 0xFF2B2622.toInt()
        private val MUTED = 0xFF6B6259.toInt()

        /** The duck's own orange, from its feet and beak. */
        private val ACCENT = 0xFFFF671F.toInt()
        private val ACCENT_DARK = 0xFFD9500F.toInt()
        private val PRESSED = 0xFFF1E4D6.toInt()
        private const val DUCK_TALL = 0.28f
        private const val FLOOR_FROM_BOTTOM = 0.04f
        private const val REQUEST_MIC = 2
        private const val REQUEST_NOTIFICATIONS = 1
    }
}
