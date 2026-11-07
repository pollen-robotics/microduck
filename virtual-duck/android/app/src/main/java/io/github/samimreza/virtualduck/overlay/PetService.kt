package io.github.samimreza.virtualduck.overlay

import android.Manifest
import android.animation.ValueAnimator
import android.annotation.SuppressLint
import android.app.KeyguardManager
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo
import android.content.res.ColorStateList
import android.content.res.Configuration
import android.graphics.Color
import android.graphics.PixelFormat
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.RippleDrawable
import android.graphics.drawable.ShapeDrawable
import android.graphics.drawable.shapes.OvalShape
import android.os.BatteryManager
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.PowerManager
import android.provider.Settings
import android.view.Gravity
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.SurfaceView
import android.view.VelocityTracker
import android.view.View
import android.view.WindowInsets
import android.view.WindowManager
import android.view.animation.DecelerateInterpolator
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView
import io.github.samimreza.virtualduck.Capture
import io.github.samimreza.virtualduck.DebugBridge
import io.github.samimreza.virtualduck.DuckAssets
import io.github.samimreza.virtualduck.DuckStage
import io.github.samimreza.virtualduck.PetHub
import io.github.samimreza.virtualduck.R
import io.github.samimreza.virtualduck.home.HomeActivity
import io.github.samimreza.virtualduck.pet.Ball
import io.github.samimreza.virtualduck.pet.Brain
import io.github.samimreza.virtualduck.pet.Listener
import io.github.samimreza.virtualduck.pet.Mood
import io.github.samimreza.virtualduck.pet.PetState
import io.github.samimreza.virtualduck.pet.ShakeSensor
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.roundToInt

/**
 * The duck on top of everything: a foreground service that owns a few small overlay windows.
 *
 *  - the duck's own window, a little bigger than the duck, which moves with it — touchable, since
 *    the duck is there to be touched, and small, since everything under it is out of reach;
 *  - a strip along its floor for seeds, not touchable and drawn at 80% opacity, which is what
 *    Android 12+ requires before it lets touches through an overlay to the app beneath;
 *  - the ball's window while there is a ball, so it can be flicked;
 *  - the care menu and the ✕ to put it away, after a long press — the ✕ also rises while the duck
 *    is held, and dropping the duck on it puts it away, the way a chat head is dismissed;
 *  - for one tap after "📍", the whole screen, to say where the duck should go.
 *
 * Its floor is a line across the screen, the bottom by default, wherever its owner puts it. The
 * duck's simulator x is metres from the left edge; [pixelsPerMeter] makes it as tall on screen as
 * its owner asked.
 */
class PetService : Service() {
    private lateinit var wm: WindowManager
    private lateinit var state: PetState
    private var stage: DuckStage? = null

    private lateinit var root: FrameLayout
    private lateinit var floor: FloorView
    private lateinit var surface: SurfaceView
    private lateinit var duckParams: WindowManager.LayoutParams
    private var seeds: SeedsView? = null
    private var ballView: BallView? = null
    private var ballParams: WindowManager.LayoutParams? = null
    private var menu: View? = null
    private var dismiss: DismissView? = null
    private var pointer: View? = null

    private var pixelsPerMeter = 600f
    private var screenWidth = 0
    private var topFloor = 0
    private var bottomFloor = 0
    private var floorY = 0
    private var windowW = 0
    private var windowH = 0
    private var floorFromBottomPx = 0
    private var sizedFor = 0
    private var flooredAt = -1f
    private val main = Handler(Looper.getMainLooper())
    private val point = FloatArray(3)
    private val screen = FloatArray(2)
    private var paused = false
    private var leaving = false
    private var seedsVersion = -1

    private val homeListener: () -> Unit = { updateVisibility() }
    private val capture = Capture(this, "overlay") { stage }
    private val ears by lazy {
        Listener(
            this,
            onStart = { stage?.brain?.listen() },
            onEnd = { heard -> stage?.brain?.heard(heard.syllables, heard.gaps) },
            onCancel = { stage?.brain?.stopListening() },
        )
    }
    private val debug = DebugBridge(this, "overlay") { stage }
    private val closePointerLater = Runnable { closePointer() }

    /** Fed from the notification before the duck was on the screen: fed once it is. */
    private var feedWhenShown = false
    private val shake by lazy { ShakeSensor(this) { sideways -> stage?.brain?.shaken(sideways) } }

    private val screenReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            when (intent.action) {
                Intent.ACTION_SCREEN_OFF -> {
                    paused = true
                    PetHub.save(this@PetService)
                }
                // Not under the lock screen, where overlays are not shown: there it waits for the unlock.
                Intent.ACTION_SCREEN_ON -> if (getSystemService(KeyguardManager::class.java)?.isKeyguardLocked != true) paused = false
                Intent.ACTION_USER_PRESENT -> paused = false
                Intent.ACTION_POWER_CONNECTED -> stage?.brain?.charging(true)
                Intent.ACTION_POWER_DISCONNECTED -> stage?.brain?.charging(false)
            }
            updateVisibility()
            // A hello from the duck the owner can see: not from this one while home is in front.
            if (intent.action == Intent.ACTION_USER_PRESENT && !PetHub.homeVisible) stage?.brain?.greetUser()
        }
    }

    private val autosave = object : Runnable {
        override fun run() {
            PetHub.save(this@PetService)
            updateNotification()
            main.postDelayed(this, 30_000L)
        }
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        wm = getSystemService(WindowManager::class.java)
        startForegroundWithNotification()
        if (!Settings.canDrawOverlays(this)) {
            stopSelf()
            return
        }
        // Before the assets: when home has already loaded them, show() runs right here.
        running = true
        DuckAssets.get(this) { assets -> show(assets) }
        registerReceiver(
            screenReceiver,
            IntentFilter().apply {
                addAction(Intent.ACTION_SCREEN_OFF)
                addAction(Intent.ACTION_SCREEN_ON)
                addAction(Intent.ACTION_USER_PRESENT)
                addAction(Intent.ACTION_POWER_CONNECTED)
                addAction(Intent.ACTION_POWER_DISCONNECTED)
            },
        )
        PetHub.listeners += homeListener
        capture.register()
        debug.register()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_FEED -> when {
                // Home is in front: the duck there is the one its owner can see.
                PetHub.homeVisible && PetHub.feedHome != null -> PetHub.feedHome?.invoke()
                stage != null -> stage?.brain?.feed()
                else -> feedWhenShown = true
            }
            ACTION_STOP -> stopSelf()
            ACTION_REFRESH -> {
                // Home changed a setting this acts on — sent while home is in front, which is when
                // the microphone may be added to what this service does.
                startForegroundWithNotification()
                applySettings()
            }
        }
        return START_STICKY
    }

    override fun onDestroy() {
        running = false
        capture.unregister()
        debug.unregister()
        PetHub.listeners -= homeListener
        runCatching { unregisterReceiver(screenReceiver) }
        main.removeCallbacksAndMessages(null)
        ears.stop()
        shake.stop()
        PetHub.save(this)
        stage?.destroy()
        stage = null
        for (view in listOfNotNull(menu, dismiss, pointer, ballView, seeds, if (this::root.isInitialized) root else null)) {
            runCatching { wm.removeView(view) }
        }
        super.onDestroy()
    }

    override fun onConfigurationChanged(newConfig: Configuration) {
        super.onConfigurationChanged(newConfig)
        if (stage != null) measureScreen()
    }

    // ---------------------------------------------------------------- windows

    private fun show(assets: DuckAssets) {
        if (!running) return
        state = PetHub.state(this)
        measureScreen()

        root = FrameLayout(this)
        floor = FloorView(this)
        surface = SurfaceView(this)
        root.addView(floor, FrameLayout.LayoutParams(windowW, windowH))
        root.addView(surface, FrameLayout.LayoutParams(windowW, windowH))
        duckParams = overlayParams(windowW, windowH, touchable = true)
        wm.addView(root, duckParams)

        val duck = DuckStage(this, surface, transparent = true, assets, state)
        duck.renderer.pixelsPerMeter = pixelsPerMeter
        duck.brain.left = EDGE
        duck.brain.right = screenWidth / pixelsPerMeter - EDGE
        // Arrives somewhere in the middle third, dropped in from just above its floor.
        duck.animator.place(screenWidth / pixelsPerMeter * (0.35f + Math.random().toFloat() * 0.3f), 0f, duck.animator.facing)
        duck.brain.dropIn()
        duck.beforeRender = ::follow
        stage = duck
        surface.setOnTouchListener(Touch())
        val battery = registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED))
        if ((battery?.getIntExtra(BatteryManager.EXTRA_PLUGGED, 0) ?: 0) != 0) duck.brain.charging(true)
        // Now that the duck's settings are known: listening, if talking is on.
        startForegroundWithNotification()
        main.postDelayed(autosave, 30_000L)
        // Restarted by the system with the screen off or locked, it waits for the owner like it would
        // have: no frames, ears or sensors in the dark.
        paused = getSystemService(PowerManager::class.java)?.isInteractive == false ||
            getSystemService(KeyguardManager::class.java)?.isKeyguardLocked == true
        updateVisibility()
        if (feedWhenShown) {
            feedWhenShown = false
            duck.brain.feed()
        }
    }

    private fun measureScreen() {
        val statusBar: Int
        if (Build.VERSION.SDK_INT >= 30) {
            val metrics = wm.currentWindowMetrics
            screenWidth = metrics.bounds.width()
            val insets = metrics.windowInsets
            val navigation = insets.getInsetsIgnoringVisibility(WindowInsets.Type.navigationBars() or WindowInsets.Type.displayCutout()).bottom
            statusBar = insets.getInsetsIgnoringVisibility(WindowInsets.Type.statusBars()).top
            bottomFloor = metrics.bounds.height() - navigation
        } else {
            @Suppress("DEPRECATION")
            val real = android.util.DisplayMetrics().also { wm.defaultDisplay.getRealMetrics(it) }
            screenWidth = real.widthPixels
            @SuppressLint("DiscouragedApi", "InternalInsetResource")
            fun dimen(name: String) = resources.getIdentifier(name, "dimen", "android").let { if (it > 0) resources.getDimensionPixelSize(it) else 0 }
            statusBar = dimen("status_bar_height")
            bottomFloor = real.heightPixels - dimen("navigation_bar_height")
        }
        sizedFor = state.sizeDp
        val was = pixelsPerMeter
        pixelsPerMeter = state.sizeDp * resources.displayMetrics.density / DUCK_TALL
        windowW = (WINDOW_W * pixelsPerMeter).roundToInt()
        windowH = (WINDOW_H * pixelsPerMeter).roundToInt()
        floorFromBottomPx = (FLOOR_FROM_BOTTOM * pixelsPerMeter).roundToInt()
        // The highest floor still keeps the duck's window under the status bar.
        topFloor = (statusBar + windowH - floorFromBottomPx).coerceAtMost(bottomFloor)
        flooredAt = state.floorLevel
        floorY = floorFor(state.floorLevel)
        stage?.let {
            it.renderer.pixelsPerMeter = pixelsPerMeter
            it.brain.left = EDGE
            it.brain.right = screenWidth / pixelsPerMeter - EDGE
            // A new size keeps the duck, its seeds and its ball where they were on the screen.
            if (was != pixelsPerMeter) {
                it.animator.place(it.animator.floorX * was / pixelsPerMeter, it.animator.floorY, it.animator.facing)
                it.brain.rescale(was / pixelsPerMeter)
            }
        }
        if (this::root.isInitialized) {
            for (child in listOf(floor, surface)) child.layoutParams = FrameLayout.LayoutParams(windowW, windowH)
            duckParams.width = windowW
            duckParams.height = windowH
            wm.updateViewLayout(root, duckParams)
            dropFloorWindows()
        }
    }

    private fun floorFor(level: Float) = (topFloor + level.coerceIn(0f, 1f) * (bottomFloor - topFloor)).roundToInt()

    /** The seed strip and ball window sit on the floor: rebuilt where it now is. */
    private fun dropFloorWindows() {
        seeds?.let { runCatching { wm.removeView(it) } }
        seeds = null
        seedsVersion = -1
        ballView?.let { runCatching { wm.removeView(it) } }
        ballView = null
    }

    /** Settings changed at home — a voice, a size, a floor, the ears — reach the overlay duck. */
    private fun applySettings() {
        val duck = stage ?: return
        duck.voice.reload()
        if (sizedFor != state.sizeDp || flooredAt != state.floorLevel) measureScreen()
        senses()
    }

    private fun overlayParams(w: Int, h: Int, touchable: Boolean) = WindowManager.LayoutParams(
        w, h,
        WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY,
        WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE or
            WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS or
            WindowManager.LayoutParams.FLAG_LAYOUT_IN_SCREEN or
            WindowManager.LayoutParams.FLAG_HARDWARE_ACCELERATED or
            (if (touchable) 0 else WindowManager.LayoutParams.FLAG_NOT_TOUCHABLE),
        PixelFormat.TRANSLUCENT,
    ).apply { gravity = Gravity.TOP or Gravity.START }

    /** After each step: the window to the duck, the camera to the window, the decorations to both. */
    private fun follow() {
        val duck = stage ?: return
        if (leaving) return
        val brain = duck.brain
        ears.deaf = duck.voice.speaking
        val x = duck.animator.floorX
        duck.renderer.frame(x, FLOOR_FROM_BOTTOM)

        val wx = (x * pixelsPerMeter - windowW / 2f).roundToInt()
        val wy = (floorY - (windowH - floorFromBottomPx) - brain.lift * pixelsPerMeter).roundToInt()
        // A window move costs a trip to the window manager; skip the frames that would not move it.
        if (wx != duckParams.x || wy != duckParams.y) {
            duckParams.x = wx
            duckParams.y = wy
            wm.updateViewLayout(root, duckParams)
        }

        val shadowWas = floor.lifted
        floor.shadowX = windowW / 2f
        floor.shadowY = (windowH - floorFromBottomPx).toFloat()
        floor.shadowWidth = 0.17f * pixelsPerMeter
        floor.lifted = (brain.lift / 0.08f).coerceIn(0f, 1f)
        val bubble = brain.bubble
        if (bubble != null || brain.hearts > 0) {
            duck.renderer.headPosition(point)
            duck.renderer.project(point[0], point[1], point[2] + 0.09f, screen)
            floor.bubbleX = screen[0].coerceIn(windowW * 0.15f, windowW * 0.85f)
            floor.bubbleY = screen[1].coerceAtLeast(windowW * 0.08f)
            if (brain.hearts > 0) {
                floor.burst(brain.hearts, screen[0], screen[1] + floor.shadowWidth * 0.3f)
                brain.hearts = 0
            }
        }
        val hearts = floor.stepHearts()
        if (bubble != floor.bubble || bubble != null || shadowWas != floor.lifted || hearts) {
            floor.bubble = bubble
            floor.invalidate()
        }

        showSeeds(brain)
        showBall(brain.ball)
    }

    private fun showSeeds(brain: Brain) {
        val pellets = brain.pellets
        if (pellets.isEmpty()) {
            seeds?.let { runCatching { wm.removeView(it) } }
            seeds = null
            return
        }
        if (seeds != null && seedsVersion == brain.pelletsVersion) return
        seedsVersion = brain.pelletsVersion
        val view = seeds ?: SeedsView(this).also { v ->
            val h = (0.03f * pixelsPerMeter).roundToInt()
            v.seedWidth = 0.022f * pixelsPerMeter
            val params = overlayParams(screenWidth, h, touchable = false).apply {
                // At or under 0.8, an untouchable overlay lets touches through on Android 12+.
                alpha = 0.8f
                y = floorY - h
            }
            wm.addView(v, params)
            seeds = v
        }
        view.seeds = FloatArray(pellets.size) { pellets[it] * pixelsPerMeter }
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun showBall(ball: Ball?) {
        if (ball == null) {
            ballView?.let { runCatching { wm.removeView(it) } }
            ballView = null
            return
        }
        val size = (Ball.RADIUS * 2.2f * pixelsPerMeter).roundToInt()
        val view = ballView ?: BallView(this).also { v ->
            val params = overlayParams(size, size, touchable = true)
            v.setOnTouchListener(BallTouch())
            wm.addView(v, params)
            ballView = v
            ballParams = params
        }
        val params = ballParams!!
        val bx = (ball.x * pixelsPerMeter - size / 2f).roundToInt()
        val by = (floorY - ball.z * pixelsPerMeter - size / 2f).roundToInt()
        view.angle = ball.angle
        if (bx != params.x || by != params.y) {
            params.x = bx
            params.y = by
            wm.updateViewLayout(view, params)
        }
    }

    private fun updateVisibility() {
        val duck = stage ?: return
        val hidden = paused || PetHub.homeVisible
        val visibility = if (hidden) View.GONE else View.VISIBLE
        root.visibility = visibility
        ballView?.visibility = visibility
        seeds?.visibility = visibility
        if (hidden) {
            duck.stop()
            dismissMenu()
            closePointer()
        } else {
            applySettings()
            // Home's duck shares its state, and may have slept or woken meanwhile.
            duck.brain.sync()
            duck.start()
        }
        senses()
    }

    /** Ears and the shake sensor run only while the duck can be seen — ears only if asked for. */
    private fun senses() {
        val visible = stage != null && !paused && !PetHub.homeVisible
        if (visible) shake.start() else shake.stop()
        if (visible && state.talk && micAllowed()) ears.start() else ears.stop()
    }

    private fun micAllowed() = checkSelfPermission(Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED

    // ---------------------------------------------------------------- the care menu and the ✕

    private fun showMenu() {
        dismissMenu()
        val duck = stage ?: return
        val density = resources.displayMetrics.density
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding((6 * density).toInt(), (4 * density).toInt(), (6 * density).toInt(), (4 * density).toInt())
            background = GradientDrawable().apply {
                cornerRadius = 28 * density
                setColor(0xF2FFFFFF.toInt())
                setStroke((1 * density).toInt(), 0x22000000)
            }
        }
        val items = listOf(
            "🌾" to { duck.brain.feed() },
            "⚽" to { duck.brain.throwBall() },
            "💃" to { duck.brain.dance() },
            "🎵" to { duck.brain.sing() },
            "🤸" to { duck.brain.trick() },
            "💤" to { duck.brain.toggleSleep() },
            "📍" to { pointWhere() },
        )
        for ((emoji, action) in items) {
            row.addView(
                TextView(this).apply {
                    text = emoji
                    textSize = 20f
                    gravity = Gravity.CENTER
                    setTextColor(Color.BLACK)
                    isSoundEffectsEnabled = false
                    val pad = (7 * density).toInt()
                    setPadding(pad, pad, pad, pad)
                    // A round ripple under the finger, so a press is seen to land.
                    background = RippleDrawable(ColorStateList.valueOf(0x40FF671F), null, ShapeDrawable(OvalShape()))
                    setOnClickListener {
                        performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
                        action()
                        // Long enough for the ripple to be seen before the menu goes.
                        main.removeCallbacks(dismissLater)
                        main.postDelayed(dismissLater, 180L)
                    }
                },
            )
        }
        row.measure(View.MeasureSpec.UNSPECIFIED, View.MeasureSpec.UNSPECIFIED)
        val params = overlayParams(
            WindowManager.LayoutParams.WRAP_CONTENT, WindowManager.LayoutParams.WRAP_CONTENT, touchable = true,
        ).apply {
            flags = flags or WindowManager.LayoutParams.FLAG_WATCH_OUTSIDE_TOUCH
            x = (duckParams.x + windowW / 2 - row.measuredWidth / 2).coerceIn(0, (screenWidth - row.measuredWidth).coerceAtLeast(0))
            y = (duckParams.y - row.measuredHeight).coerceAtLeast(0)
        }
        row.setOnTouchListener { _, event ->
            if (event.action == MotionEvent.ACTION_OUTSIDE) dismissMenu()
            false
        }
        row.alpha = 0f
        row.scaleX = 0.8f
        row.scaleY = 0.8f
        wm.addView(row, params)
        row.animate().alpha(1f).scaleX(1f).scaleY(1f).setDuration(150).setInterpolator(DecelerateInterpolator()).start()
        menu = row
        showDismiss(tappable = true)
        main.postDelayed(dismissLater, 8_000L)
    }

    private val dismissLater = Runnable { dismissMenu() }

    private fun dismissMenu() {
        main.removeCallbacks(dismissLater)
        menu?.let { runCatching { wm.removeView(it) } }
        menu = null
        hideDismiss()
    }

    /** Where the ✕ sits: centred, a little above the bottom of the screen — where chat heads go. */
    private fun dismissCenter(): Pair<Int, Int> =
        screenWidth / 2 to (bottomFloor - (DISMISS_LIFT_DP * resources.displayMetrics.density)).roundToInt()

    private val dismissSize get() = (DISMISS_DP * resources.displayMetrics.density).roundToInt()

    @SuppressLint("ClickableViewAccessibility")
    private fun showDismiss(tappable: Boolean) {
        if (dismiss != null) return
        val size = dismissSize
        val (cx, cy) = dismissCenter()
        val view = DismissView(this)
        val params = overlayParams(size, size, touchable = tappable).apply {
            x = cx - size / 2
            y = cy - size / 2
        }
        if (tappable) {
            view.isSoundEffectsEnabled = false
            view.setOnClickListener {
                view.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS)
                putAway()
            }
        }
        view.alpha = 0f
        view.translationY = size * 0.6f
        wm.addView(view, params)
        view.animate().alpha(1f).translationY(0f).setDuration(180).setInterpolator(DecelerateInterpolator()).start()
        dismiss = view
    }

    private fun hideDismiss() {
        val view = dismiss ?: return
        dismiss = null
        view.animate().alpha(0f).translationY(view.height * 0.6f).setDuration(150).withEndAction {
            runCatching { wm.removeView(view) }
        }.start()
    }

    /**
     * Put away: the duck shrinks into the ✕ — its window fading as it slides there — says a soft
     * goodbye, and the service ends. Home can let it out again.
     */
    private fun putAway() {
        if (leaving) return
        leaving = true
        main.removeCallbacks(dismissLater)
        menu?.let { runCatching { wm.removeView(it) } }
        menu = null
        stage?.brain?.putAway()
        val (cx, cy) = dismissCenter()
        val fromX = duckParams.x
        val fromY = duckParams.y
        val toX = cx - windowW / 2
        val toY = cy - windowH / 2
        ValueAnimator.ofFloat(0f, 1f).apply {
            duration = 260
            addUpdateListener {
                val t = it.animatedValue as Float
                duckParams.x = (fromX + (toX - fromX) * t).roundToInt()
                duckParams.y = (fromY + (toY - fromY) * t).roundToInt()
                duckParams.alpha = 1f - t
                runCatching { wm.updateViewLayout(root, duckParams) }
            }
            withEnd { stopSelf() }
            start()
        }
        hideDismiss()
    }

    private fun ValueAnimator.withEnd(action: () -> Unit) {
        addListener(object : android.animation.AnimatorListenerAdapter() {
            override fun onAnimationEnd(animation: android.animation.Animator) = action()
        })
    }

    // ---------------------------------------------------------------- "📍": where should it go

    /**
     * The next tap anywhere says where the duck goes. On its floor it walks there; anywhere higher or
     * lower it jumps there, and that becomes its floor — how its owner moves where it lives.
     */
    @SuppressLint("ClickableViewAccessibility")
    private fun pointWhere() {
        if (pointer != null) return
        val view = PointView(this, getString(R.string.point_hint, state.name))
        val params = overlayParams(WindowManager.LayoutParams.MATCH_PARENT, WindowManager.LayoutParams.MATCH_PARENT, touchable = true)
        view.setOnTouchListener { _, event ->
            if (event.actionMasked == MotionEvent.ACTION_UP) {
                closePointer()
                goTo(event.rawX, event.rawY)
            }
            true
        }
        wm.addView(view, params)
        pointer = view
        // Changed its mind: the pointer goes after a while untouched.
        main.postDelayed(closePointerLater, 8_000L)
    }

    private fun closePointer() {
        main.removeCallbacks(closePointerLater)
        pointer?.let { runCatching { wm.removeView(it) } }
        pointer = null
    }

    private fun goTo(rawX: Float, rawY: Float) {
        val duck = stage ?: return
        val x = rawX / pixelsPerMeter
        val newFloor = rawY.roundToInt().coerceIn(topFloor, bottomFloor)
        if (abs(newFloor - floorY) < 30 * resources.displayMetrics.density) {
            duck.brain.come(x)
            return
        }
        // Its feet, measured from the new floor; then the floor moves and it jumps to it.
        val from = (newFloor - floorY) / pixelsPerMeter + duck.brain.lift
        floorY = newFloor
        state.floorLevel = if (bottomFloor > topFloor) (newFloor - topFloor).toFloat() / (bottomFloor - topFloor) else 1f
        flooredAt = state.floorLevel
        dropFloorWindows()
        duck.brain.jump(x, from)
    }

    // ---------------------------------------------------------------- touch

    /**
     * Tap: a poke (a double tap: a dance). Rub back and forth over it: petting. Drag: picked up — the
     * ✕ rises, and dropping it there puts it away — and let go elsewhere: thrown. Hold still: the
     * care menu. Told apart by how far the finger has gone against how far it has got: a rub covers
     * ground without getting anywhere.
     */
    private inner class Touch : View.OnTouchListener {
        private val slop = 12 * resources.displayMetrics.density
        private var mode = Mode.NONE
        private var downX = 0f
        private var downY = 0f
        private var lastX = 0f
        private var lastY = 0f
        private var lastTime = 0L
        private var downTime = 0L
        private var path = 0f
        private var stroked = 0f
        private var offsetX = 0f
        private var offsetY = 0f
        private var lastTap = 0L
        private var tracker: VelocityTracker? = null
        private val longPress = Runnable {
            if (mode == Mode.UNDECIDED && root.visibility == View.VISIBLE) {
                mode = Mode.NONE
                surface.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS)
                showMenu()
            }
        }

        @SuppressLint("ClickableViewAccessibility")
        override fun onTouch(v: View, event: MotionEvent): Boolean {
            val duck = stage ?: return false
            if (leaving) return true
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    mode = if (onDuck(event.x, event.y)) Mode.UNDECIDED else Mode.NONE
                    downX = event.rawX
                    downY = event.rawY
                    lastX = downX
                    lastY = downY
                    downTime = event.eventTime
                    lastTime = downTime
                    path = 0f
                    stroked = 0f
                    tracker?.recycle()
                    tracker = VelocityTracker.obtain().also { it.addRaw(event) }
                    if (mode == Mode.UNDECIDED) main.postDelayed(longPress, 450L)
                }
                MotionEvent.ACTION_MOVE -> {
                    tracker?.addRaw(event)
                    val step = hypot(event.rawX - lastX, event.rawY - lastY)
                    path += step
                    val net = hypot(event.rawX - downX, event.rawY - downY)
                    when (mode) {
                        Mode.UNDECIDED -> if (net > slop || path > slop) {
                            main.removeCallbacks(longPress)
                            if (path > 2.2f * net && path > 5 * slop) {
                                mode = Mode.STROKING
                            } else if (net > 4 * slop && path < 1.5f * net) {
                                mode = Mode.CARRYING
                                dismissMenu()
                                offsetX = event.rawX - duckParams.x
                                offsetY = event.rawY - duckParams.y
                                duck.brain.grab(duck.animator.floorX)
                                showDismiss(tappable = false)
                            }
                        }
                        Mode.STROKING -> {
                            stroked += step
                            if (stroked > 6 * slop) {
                                stroked = 0f
                                duck.renderer.unproject(event.x, event.y, point)
                                duck.brain.stroke(point[0], point[2])
                            }
                        }
                        Mode.CARRYING -> carry(duck, event)
                        Mode.NONE -> Unit
                    }
                    lastX = event.rawX
                    lastY = event.rawY
                    lastTime = event.eventTime
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    main.removeCallbacks(longPress)
                    tracker?.addRaw(event)
                    when (mode) {
                        Mode.CARRYING -> {
                            if (overDismiss(event.rawX, event.rawY)) {
                                putAway()
                            } else {
                                val t = tracker!!
                                t.computeCurrentVelocity(1000)
                                duck.brain.release(t.xVelocity / pixelsPerMeter, -t.yVelocity / pixelsPerMeter)
                                hideDismiss()
                            }
                        }
                        Mode.UNDECIDED -> if (event.actionMasked == MotionEvent.ACTION_UP && event.eventTime - downTime < 350L) {
                            if (event.eventTime - lastTap < 320L) {
                                duck.brain.dance()
                                lastTap = 0L
                            } else {
                                duck.renderer.unproject(event.x, event.y, point)
                                duck.brain.poke(point[0], point[2])
                                lastTap = event.eventTime
                            }
                        }
                        else -> Unit
                    }
                    mode = Mode.NONE
                    tracker?.recycle()
                    tracker = null
                }
            }
            return true
        }

        /**
         * Follows the finger — unless it is over the ✕, which pulls the duck onto itself the way a
         * chat head snaps to its target, with a tick as it catches.
         */
        private fun carry(duck: DuckStage, event: MotionEvent) {
            val dt = (event.eventTime - lastTime) / 1000f
            val over = overDismiss(event.rawX, event.rawY)
            val target = dismiss
            if (target != null && over != target.hot) {
                target.hot = over
                if (over) target.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            }
            val (cx, cy) = dismissCenter()
            val wx = if (over) cx - windowW / 2f else event.rawX - offsetX
            val wy = if (over) cy - windowH / 2f else event.rawY - offsetY
            val restY = floorY - (windowH - floorFromBottomPx)
            duck.brain.carry((wx + windowW / 2f) / pixelsPerMeter, (restY - wy) / pixelsPerMeter, dt)
        }

        /** Is the finger close enough to the ✕ for it to catch the duck? */
        private fun overDismiss(rawX: Float, rawY: Float): Boolean {
            if (dismiss == null) return false
            val (cx, cy) = dismissCenter()
            return hypot(rawX - cx, rawY - cy) < dismissSize * 1.1f
        }

        /** Is (x, y), in window pixels, on the duck and not the empty corners of its window? */
        private fun onDuck(x: Float, y: Float): Boolean {
            val duck = stage ?: return false
            val animator = duck.animator
            duck.renderer.project(animator.floorX, animator.floorY, animator.height, screen)
            if (hypot(x - screen[0], y - screen[1]) < 0.11f * pixelsPerMeter) return true
            duck.renderer.headPosition(point)
            duck.renderer.project(point[0], point[1], point[2], screen)
            if (hypot(x - screen[0], y - screen[1]) < 0.09f * pixelsPerMeter) return true
            // Legs: anywhere between the trunk and the floor, near the middle.
            return abs(x - windowW / 2f) < 0.07f * pixelsPerMeter && y > windowH * 0.4f
        }
    }

    private enum class Mode { NONE, UNDECIDED, STROKING, CARRYING }

    /**
     * Adds [event] in screen coordinates. The duck's window follows the finger carrying it, so in
     * the window's own coordinates a throw hardly moves at all.
     */
    private fun VelocityTracker.addRaw(event: MotionEvent) {
        val dx = event.rawX - event.x
        val dy = event.rawY - event.y
        event.offsetLocation(dx, dy)
        addMovement(event)
        event.offsetLocation(-dx, -dy)
    }

    /** The ball can be flicked along the floor. */
    private inner class BallTouch : View.OnTouchListener {
        private var tracker: VelocityTracker? = null

        @SuppressLint("ClickableViewAccessibility")
        override fun onTouch(v: View, event: MotionEvent): Boolean {
            when (event.actionMasked) {
                // In screen coordinates: the ball's window moves with the ball.
                MotionEvent.ACTION_DOWN -> {
                    tracker?.recycle()
                    tracker = VelocityTracker.obtain().also { it.addRaw(event) }
                }
                MotionEvent.ACTION_MOVE -> tracker?.addRaw(event)
                MotionEvent.ACTION_CANCEL -> {
                    tracker?.recycle()
                    tracker = null
                }
                MotionEvent.ACTION_UP -> {
                    tracker?.let {
                        it.addRaw(event)
                        it.computeCurrentVelocity(1000)
                        val vx = it.xVelocity / pixelsPerMeter
                        stage?.brain?.flickBall(if (abs(vx) < 0.2f) 0.8f else vx)
                        it.recycle()
                    }
                    tracker = null
                }
            }
            return true
        }
    }

    // ---------------------------------------------------------------- notification

    /**
     * Foreground, as a special use — a pet that stays on screen — and as a microphone user too when
     * talking is on and allowed: Android 14+ lets a service listen only if it says so, and only when
     * it says so while the app is in front, which is when home sends [ACTION_REFRESH].
     */
    private fun startForegroundWithNotification() {
        val manager = getSystemService(NotificationManager::class.java)
        manager.createNotificationChannel(
            NotificationChannel(CHANNEL, getString(R.string.channel_name), NotificationManager.IMPORTANCE_LOW).apply {
                setShowBadge(false)
            },
        )
        val notification = buildNotification()
        val listening = this::state.isInitialized && state.talk && micAllowed()
        when {
            Build.VERSION.SDK_INT >= 34 -> {
                val types = ServiceInfo.FOREGROUND_SERVICE_TYPE_SPECIAL_USE or
                    (if (listening) ServiceInfo.FOREGROUND_SERVICE_TYPE_MICROPHONE else 0)
                runCatching { startForeground(NOTIFICATION, notification, types) }
                    .onFailure { startForeground(NOTIFICATION, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_SPECIAL_USE) }
            }
            Build.VERSION.SDK_INT >= 30 && listening ->
                runCatching { startForeground(NOTIFICATION, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_MICROPHONE) }
                    .onFailure { startForeground(NOTIFICATION, notification) }
            else -> startForeground(NOTIFICATION, notification)
        }
    }

    private fun updateNotification() {
        getSystemService(NotificationManager::class.java).notify(NOTIFICATION, buildNotification())
    }

    private fun buildNotification(): Notification {
        val flags = PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
        val home = PendingIntent.getActivity(this, 0, Intent(this, HomeActivity::class.java), flags)
        val feed = PendingIntent.getService(this, 1, Intent(this, PetService::class.java).setAction(ACTION_FEED), flags)
        val stop = PendingIntent.getService(this, 2, Intent(this, PetService::class.java).setAction(ACTION_STOP), flags)
        val name = if (this::state.isInitialized) state.name else getString(R.string.app_name)
        val mood = if (this::state.isInitialized) moodLine(state) else ""
        return Notification.Builder(this, CHANNEL)
            .setSmallIcon(R.drawable.ic_stat_duck)
            .setContentTitle(getString(R.string.notification_title, name))
            .setContentText(mood)
            .setContentIntent(home)
            .setOngoing(true)
            .addAction(Notification.Action.Builder(null, getString(R.string.feed), feed).build())
            .addAction(Notification.Action.Builder(null, getString(R.string.put_away), stop).build())
            .build()
    }

    private fun moodLine(pet: PetState) = getString(
        when (pet.mood) {
            Mood.HAPPY -> R.string.mood_happy
            Mood.CONTENT -> R.string.mood_content
            Mood.HUNGRY -> R.string.mood_hungry
            Mood.SLEEPY -> R.string.mood_sleepy
            Mood.LONELY -> R.string.mood_lonely
            Mood.BORED -> R.string.mood_bored
        },
    )

    companion object {
        const val ACTION_FEED = "io.github.samimreza.virtualduck.FEED"
        const val ACTION_STOP = "io.github.samimreza.virtualduck.STOP"
        const val ACTION_REFRESH = "io.github.samimreza.virtualduck.REFRESH"
        private const val CHANNEL = "duck"
        private const val NOTIFICATION = 1

        /** Top of the head above the floor, standing — what "the duck is this tall" measures. */
        const val DUCK_TALL = 0.28f

        /** The duck's window, metres: room for it to fall over in, and a thought over its head. */
        const val WINDOW_W = 0.44f
        const val WINDOW_H = 0.38f
        const val FLOOR_FROM_BOTTOM = 0.03f

        /** How close to the screen's edges the duck's trunk may walk. */
        const val EDGE = 0.08f

        /** The ✕: its size, and how far above the bottom of the screen it sits. */
        const val DISMISS_DP = 64
        const val DISMISS_LIFT_DP = 110

        @Volatile
        var running = false
            private set

        fun start(context: Context) {
            context.startForegroundService(Intent(context, PetService::class.java))
        }

        fun stop(context: Context) {
            context.stopService(Intent(context, PetService::class.java))
        }
    }
}
