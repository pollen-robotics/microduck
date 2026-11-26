package io.github.samimreza.virtualduck.overlay

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RadialGradient
import android.graphics.RectF
import android.graphics.Shader
import android.view.View

// The flat things around the duck. In an orthographic view from the front a shadow is an ellipse
// and a ball is a shaded disc, so drawing them in 2D costs a few canvas calls instead of a second
// 3D scene, and looks the same.

/** Under the duck: its shadow, and a thought bubble over its head. Drawn behind the 3D surface. */
class FloorView(context: Context) : View(context) {
    /** Window pixels: where the floor meets the duck's trunk. */
    var shadowX = 0f
    var shadowY = 0f
    var shadowWidth = 100f
    /** 0 on the floor, 1 lifted far enough that the shadow has gone. */
    var lifted = 0f

    var bubble: String? = null
    var bubbleX = 0f
    var bubbleY = 0f

    private val shadow = Paint(Paint.ANTI_ALIAS_FLAG)
    private val card = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE }
    private val outline = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0x33000000
        style = Paint.Style.STROKE
        strokeWidth = 2f
    }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textAlign = Paint.Align.CENTER }
    private val oval = RectF()
    private var shadowFor = -1f

    private class Heart(var x: Float, var y: Float, val drift: Float, var age: Float)

    private val hearts = ArrayList<Heart>()
    private val heartPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { textAlign = Paint.Align.CENTER }
    private var heartsAt = 0L

    /** Hearts float up from (x, y) — a stroke, a level, a happy wiggle. */
    fun burst(count: Int, x: Float, y: Float) {
        repeat(count.coerceAtMost(8)) {
            hearts += Heart(x + (Math.random().toFloat() - 0.5f) * shadowWidth * 0.5f, y, (Math.random().toFloat() - 0.5f) * 2f, -it * 0.12f)
        }
        heartsAt = android.os.SystemClock.uptimeMillis()
    }

    /** Moves the hearts on; false once there are none left to draw. */
    fun stepHearts(): Boolean {
        if (hearts.isEmpty()) return false
        val now = android.os.SystemClock.uptimeMillis()
        val dt = ((now - heartsAt) / 1000f).coerceAtMost(0.1f)
        heartsAt = now
        val rise = shadowWidth * 0.9f
        val iterator = hearts.iterator()
        while (iterator.hasNext()) {
            val heart = iterator.next()
            heart.age += dt
            if (heart.age > HEART_LIFE) iterator.remove()
            if (heart.age > 0f) {
                heart.y -= rise * dt
                heart.x += heart.drift * rise * 0.25f * dt
            }
        }
        return true
    }

    override fun onDraw(canvas: Canvas) {
        if (lifted < 1f) {
            if (shadowFor != shadowWidth) {
                // A unit radial gradient, stretched to the ellipse by the oval it fills.
                shadow.shader = RadialGradient(0f, 0f, 1f, 0x55000000, 0x00000000, Shader.TileMode.CLAMP)
                shadowFor = shadowWidth
            }
            val w = shadowWidth * (1f - 0.5f * lifted)
            val h = w * 0.22f
            canvas.save()
            canvas.translate(shadowX, shadowY)
            canvas.scale(w / 2, h / 2)
            shadow.alpha = (255 * (1f - lifted)).toInt()
            oval.set(-1f, -1f, 1f, 1f)
            canvas.drawOval(oval, shadow)
            canvas.restore()
        }
        bubble?.let { emoji ->
            val size = shadowWidth * 0.32f
            text.textSize = size
            val r = size * 0.85f
            oval.set(bubbleX - r, bubbleY - r * 0.85f, bubbleX + r, bubbleY + r * 0.85f)
            canvas.drawRoundRect(oval, r, r, card)
            canvas.drawRoundRect(oval, r, r, outline)
            canvas.drawText(emoji, bubbleX, bubbleY + size * 0.35f, text)
        }
        heartPaint.textSize = shadowWidth * 0.22f
        for (heart in hearts) {
            if (heart.age < 0f) continue
            val life = heart.age / HEART_LIFE
            heartPaint.alpha = (255 * (1f - life * life)).toInt()
            canvas.drawText("❤️", heart.x, heart.y, heartPaint)
        }
    }

    companion object {
        const val HEART_LIFE = 1.3f
    }
}

/**
 * The target a held duck is dropped on to put it away — the way a chat head is dismissed: a round
 * ✕ that rises from the bottom while the duck is held, and grows and reddens when the duck is over
 * it. Shown with the long-press menu too, where tapping it does the same.
 */
class DismissView(context: Context) : View(context) {
    var hot = false
        set(value) {
            if (value != field) {
                field = value
                animate().scaleX(if (value) 1.25f else 1f).scaleY(if (value) 1.25f else 1f).setDuration(120).start()
                invalidate()
            }
        }

    private val disc = Paint(Paint.ANTI_ALIAS_FLAG)
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        color = 0x66FFFFFF
    }
    private val cross = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        strokeCap = Paint.Cap.ROUND
    }

    override fun onDraw(canvas: Canvas) {
        val r = width / 2f * 0.8f
        val cx = width / 2f
        val cy = height / 2f
        disc.color = if (hot) 0xE6E53935.toInt() else 0xB3202124.toInt()
        canvas.drawCircle(cx, cy, r, disc)
        ring.strokeWidth = r * 0.06f
        canvas.drawCircle(cx, cy, r, ring)
        cross.strokeWidth = r * 0.14f
        val a = r * 0.36f
        canvas.drawLine(cx - a, cy - a, cx + a, cy + a, cross)
        canvas.drawLine(cx - a, cy + a, cx + a, cy - a, cross)
    }
}

/**
 * The whole screen, for one tap: "tap where your duck should go". Lightly dimmed so it is clear the
 * next tap is the duck's, not the app's underneath.
 */
class PointView(context: Context, hint: String) : View(context) {
    private val label = hint
    private val shade = Paint().apply { color = 0x26000000 }
    private val chip = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xF2FFFFFF.toInt() }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFF2B2622.toInt()
        textAlign = Paint.Align.CENTER
        textSize = 16 * context.resources.displayMetrics.scaledDensity
    }
    private val box = RectF()

    override fun onDraw(canvas: Canvas) {
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), shade)
        val d = resources.displayMetrics.density
        val w = text.measureText(label) + 40 * d
        box.set(width / 2f - w / 2, height * 0.12f, width / 2f + w / 2, height * 0.12f + 44 * d)
        canvas.drawRoundRect(box, 22 * d, 22 * d, chip)
        canvas.drawText(label, width / 2f, box.centerY() + text.textSize * 0.35f, text)
    }
}

/** The kick ball, rolling: a shaded disc with a stripe so it can be seen to turn. */
class BallView(context: Context) : View(context) {
    var angle = 0f
        set(value) {
            if (value != field) {
                field = value
                invalidate()
            }
        }

    private val body = Paint(Paint.ANTI_ALIAS_FLAG)
    private val stripe = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = 0xFFFFFFFF.toInt()
        style = Paint.Style.STROKE
    }
    private var sizedFor = -1

    override fun onDraw(canvas: Canvas) {
        val r = width / 2f * 0.9f
        if (sizedFor != width) {
            body.shader = RadialGradient(
                width / 2f - r * 0.35f, height / 2f - r * 0.35f, r * 1.4f,
                0xFFFFB27A.toInt(), 0xFFE2561E.toInt(), Shader.TileMode.CLAMP,
            )
            stripe.strokeWidth = r * 0.18f
            sizedFor = width
        }
        canvas.drawCircle(width / 2f, height / 2f, r, body)
        canvas.save()
        canvas.rotate(Math.toDegrees(angle.toDouble()).toFloat(), width / 2f, height / 2f)
        canvas.drawLine(width / 2f - r * 0.85f, height / 2f, width / 2f + r * 0.85f, height / 2f, stripe)
        canvas.restore()
    }
}

/** Seeds on the floor, along the bottom of the screen. */
class SeedsView(context: Context) : View(context) {
    /** Screen x of each seed, pixels. */
    var seeds: FloatArray = FloatArray(0)
        set(value) {
            if (!value.contentEquals(field)) {
                field = value
                invalidate()
            }
        }
    var seedWidth = 14f

    private val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFFD9A441.toInt() }
    private val oval = RectF()

    override fun onDraw(canvas: Canvas) {
        val h = seedWidth * 0.6f
        for (x in seeds) {
            oval.set(x - seedWidth / 2, height - h * 1.2f, x + seedWidth / 2, height - h * 0.2f)
            canvas.drawOval(oval, paint)
        }
    }
}
