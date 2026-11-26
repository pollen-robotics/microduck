package io.github.samimreza.virtualduck.render

import android.graphics.Bitmap
import android.graphics.PixelFormat
import android.os.Handler
import android.os.Looper
import android.view.Surface
import android.view.SurfaceView
import com.google.android.filament.Camera
import com.google.android.filament.Engine
import com.google.android.filament.EntityManager
import com.google.android.filament.Filament
import com.google.android.filament.IndirectLight
import com.google.android.filament.LightManager
import com.google.android.filament.Renderer
import com.google.android.filament.Skybox
import com.google.android.filament.SwapChain
import com.google.android.filament.Texture
import com.google.android.filament.View
import com.google.android.filament.Viewport
import com.google.android.filament.android.UiHelper
import com.google.android.filament.gltfio.AssetLoader
import com.google.android.filament.gltfio.FilamentAsset
import com.google.android.filament.gltfio.Gltfio
import com.google.android.filament.gltfio.ResourceLoader
import com.google.android.filament.gltfio.UbershaderProvider
import io.github.samimreza.virtualduck.motion.Body
import io.github.samimreza.virtualduck.motion.Rig
import io.github.samimreza.virtualduck.motion.TRUNK
import java.nio.ByteBuffer
import kotlin.math.cos
import kotlin.math.sin

/**
 * The duck, drawn by Filament into a SurfaceView — transparent over other apps, or on a colour at
 * home — with an orthographic camera looking at it from the front, a little from above.
 *
 * Everything the camera and the poses need is allocated once. A frame is one transform
 * transaction — fifteen local matrices, committed together so world transforms are resolved once —
 * and one render.
 */
class DuckRenderer(
    surfaceView: SurfaceView,
    private val transparent: Boolean,
    glb: ByteBuffer,
    rig: Rig,
    jointOrder: List<String>,
    /** Linear RGBA behind the duck when it is not transparent: the home screen's warm cream. */
    background: DoubleArray = doubleArrayOf(0.84, 0.77, 0.65, 1.0),
) : Body {
    private val engine = Engine.create()
    private val renderer = engine.createRenderer()
    private val scene = engine.createScene()
    private val view = engine.createView()
    private val cameraEntity = EntityManager.get().create()
    private val camera = engine.createCamera(cameraEntity)
    private val sun = EntityManager.get().create()
    private val indirect: IndirectLight
    private val skybox: Skybox?
    private val uiHelper = UiHelper(UiHelper.ContextErrorPolicy.DONT_CHECK)
    private var swapChain: SwapChain? = null
    private val materials = UbershaderProvider(engine)
    private val loader = AssetLoader(engine, materials, EntityManager.get())
    private val resources = ResourceLoader(engine)
    private val asset: FilamentAsset

    private val trunk: Int
    private val head: Int
    /** Per joint in the animator's order: transform instance, rest matrix, axis, range. */
    private val joints: IntArray
    private val jointSlot: IntArray
    private val rest: Array<FloatArray>
    private val axes: Array<FloatArray>
    private val ranges: Array<FloatArray>
    private val mouthParts: IntArray
    private val mouthRest: Array<FloatArray>
    private val mouthPivot: FloatArray
    private val mouthAxis: FloatArray

    private val m = FloatArray(16)
    private val r = FloatArray(16)
    private val world = FloatArray(16)

    /** Screen pixels per simulator metre. */
    var pixelsPerMeter = 600f
    var widthPx = 1
        private set
    var heightPx = 1
        private set
    private var centerX = 0f
    private var floorFromBottom = 0.03f
    private var zc = 0f

    init {
        if (transparent) {
            surfaceView.setZOrderOnTop(true)
            surfaceView.holder.setFormat(PixelFormat.TRANSLUCENT)
        }
        uiHelper.isOpaque = !transparent
        uiHelper.renderCallback = object : UiHelper.RendererCallback {
            override fun onNativeWindowChanged(surface: Surface) {
                swapChain?.let { engine.destroySwapChain(it) }
                swapChain = engine.createSwapChain(surface, uiHelper.swapChainFlags)
            }

            override fun onDetachedFromSurface() {
                swapChain?.let {
                    engine.destroySwapChain(it)
                    engine.flushAndWait()
                }
                swapChain = null
            }

            override fun onResized(width: Int, height: Int) {
                widthPx = width
                heightPx = height
                view.viewport = Viewport(0, 0, width, height)
                frame(centerX, floorFromBottom)
            }
        }

        view.scene = scene
        view.camera = camera
        view.blendMode = if (transparent) View.BlendMode.TRANSLUCENT else View.BlendMode.OPAQUE
        // Mobile settings: no shadow maps, no screen-space effects; 4× MSAA is close to free on the
        // tiled GPUs phones have, and a small, mostly empty surface is all there is to fill.
        view.setShadowingEnabled(false)
        view.antiAliasing = View.AntiAliasing.NONE
        view.multiSampleAntiAliasingOptions = view.multiSampleAntiAliasingOptions.apply {
            enabled = true
            sampleCount = 4
        }
        view.renderQuality = view.renderQuality.apply { hdrColorBuffer = View.QualityLevel.MEDIUM }
        renderer.clearOptions = renderer.clearOptions.apply {
            clear = true
            clearColor = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
        }
        // An opaque view starts every frame from black, whatever the renderer clears to; a plain
        // colour behind the duck is a skybox. The overlay has none, and stays see-through.
        skybox = if (transparent) {
            null
        } else {
            Skybox.Builder()
                .color(background[0].toFloat(), background[1].toFloat(), background[2].toFloat(), 1f)
                .build(engine)
        }
        scene.skybox = skybox
        camera.setExposure(16f, 1f / 125f, 100f)

        LightManager.Builder(LightManager.Type.DIRECTIONAL)
            .color(1f, 0.96f, 0.9f)
            .intensity(95_000f)
            .direction(0.4f, -1f, -0.7f)
            .castShadows(false)
            .build(engine, sun)
        scene.addEntity(sun)
        indirect = IndirectLight.Builder()
            .irradiance(1, floatArrayOf(0.95f, 0.95f, 1f))
            .intensity(32_000f)
            .build(engine)
        scene.indirectLight = indirect

        asset = loader.createAsset(glb) ?: error("duck.glb did not load")
        resources.loadResources(asset)
        asset.releaseSourceData()
        scene.addEntities(asset.entities)

        val tm = engine.transformManager
        fun instance(name: String): Int {
            val entity = asset.getFirstEntityByName(name)
            check(entity != 0) { "no node called $name in duck.glb" }
            return tm.getInstance(entity)
        }
        fun local(instance: Int) = FloatArray(16).also { tm.getTransform(instance, it) }

        trunk = instance(rig.trunk)
        head = instance(rig.joints.first { it.name == "head_roll" }.node)
        val named = rig.joints.filter { it.name in jointOrder }
        joints = IntArray(named.size) { instance(named[it].node) }
        jointSlot = IntArray(named.size) { TRUNK + jointOrder.indexOf(named[it].name) }
        rest = Array(named.size) { local(joints[it]) }
        axes = Array(named.size) { named[it].axis.let { a -> floatArrayOf(a.x, a.y, a.z) } }
        ranges = Array(named.size) { floatArrayOf(named[it].min, named[it].max) }
        mouthParts = IntArray(rig.mouthParts.size) { instance(rig.mouthParts[it]) }
        mouthRest = Array(mouthParts.size) { local(mouthParts[it]) }
        mouthPivot = rig.mouthPivot.let { floatArrayOf(it.x, it.y, it.z) }
        mouthAxis = rig.mouthAxis.let { floatArrayOf(it.x, it.y, it.z) }

        uiHelper.attachTo(surfaceView)
    }

    /**
     * Points the camera: centred on simulator x [x], with the floor [floorFromBottomMeters] above
     * the view's bottom edge. The camera looks along +Y from a little above — enough to see the
     * feet meet the floor — and Filament's world is the simulator's turned Z-up to Y-up: (x, z, -y).
     */
    fun frame(x: Float, floorFromBottomMeters: Float) {
        centerX = x
        floorFromBottom = floorFromBottomMeters
        val w = widthPx / pixelsPerMeter
        val h = heightPx / pixelsPerMeter
        zc = (h / 2 - floorFromBottomMeters) / COS_ELEVATION
        camera.setProjection(Camera.Projection.ORTHO, -w / 2.0, w / 2.0, -h / 2.0, h / 2.0, 0.05, 6.0)
        // Eye two metres back along (0, -cos, sin) from the centre; up is (0, sin, cos).
        val ey = -2f * COS_ELEVATION
        val ez = zc + 2f * SIN_ELEVATION
        camera.lookAt(
            x.toDouble(), ez.toDouble(), -ey.toDouble(),
            x.toDouble(), zc.toDouble(), 0.0,
            0.0, COS_ELEVATION.toDouble(), -SIN_ELEVATION.toDouble(),
        )
    }

    /** A simulator point to view pixels, into [out]: the same orthographic projection the camera uses. */
    fun project(x: Float, y: Float, z: Float, out: FloatArray) {
        val u = x - centerX
        val v = y * SIN_ELEVATION + (z - zc) * COS_ELEVATION
        out[0] = widthPx / 2f + u * pixelsPerMeter
        out[1] = heightPx / 2f - v * pixelsPerMeter
    }

    /** View pixels back to a simulator point on the plane through the duck (y = 0), into [out]. */
    fun unproject(px: Float, py: Float, out: FloatArray) {
        out[0] = centerX + (px - widthPx / 2f) / pixelsPerMeter
        out[1] = 0f
        out[2] = zc + (heightPx / 2f - py) / pixelsPerMeter / COS_ELEVATION
    }

    /** Where the head is, in the simulator's frame, into [out]. */
    fun headPosition(out: FloatArray) {
        engine.transformManager.getWorldTransform(head, world)
        out[0] = world[12]
        out[1] = -world[14]
        out[2] = world[13]
    }

    override fun setPose(frame: FloatArray, mouth: Float) {
        val tm = engine.transformManager
        tm.openLocalTransformTransaction()
        quatPosition(frame[3], frame[4], frame[5], frame[6], frame[0], frame[1], frame[2], m)
        tm.setTransform(trunk, m)
        for (i in joints.indices) {
            val angle = frame[jointSlot[i]].coerceIn(ranges[i][0], ranges[i][1])
            axisAngle(axes[i], angle, r)
            android.opengl.Matrix.multiplyMM(m, 0, rest[i], 0, r, 0)
            tm.setTransform(joints[i], m)
        }
        // The beak hinges about its pivot in the head's frame: H = T(p) R T(-p), H·rest.
        axisAngle(mouthAxis, mouth.coerceIn(0f, 1f) * MOUTH_OPEN, r)
        for (k in 0 until 3) {
            r[12 + k] = mouthPivot[k] - (r[k] * mouthPivot[0] + r[4 + k] * mouthPivot[1] + r[8 + k] * mouthPivot[2])
        }
        for (i in mouthParts.indices) {
            android.opengl.Matrix.multiplyMM(m, 0, r, 0, mouthRest[i], 0)
            tm.setTransform(mouthParts[i], m)
        }
        tm.commitLocalTransformTransaction()
    }

    /** Draws a frame, if there is a surface to draw on. */
    fun render(frameTimeNanos: Long) {
        val chain = swapChain ?: return
        if (!uiHelper.isReadyToRender) return
        if (renderer.beginFrame(chain, frameTimeNanos)) {
            renderer.render(view)
            captureNext?.let { readBack(it) }
            captureNext = null
            renderer.endFrame()
        }
    }

    private var captureNext: ((Bitmap) -> Unit)? = null

    /**
     * Hands [done] the next frame as Filament drew it — read back from the renderer, not the screen,
     * so it is the duck's own pixels whatever the compositor does with them. For screenshots.
     */
    fun capture(done: (Bitmap) -> Unit) {
        captureNext = done
    }

    private fun readBack(done: (Bitmap) -> Unit) {
        val w = widthPx
        val h = heightPx
        val pixels = ByteBuffer.allocateDirect(w * h * 4)
        val descriptor = Texture.PixelBufferDescriptor(
            pixels, Texture.Format.RGBA, Texture.Type.UBYTE, 1, 0, 0, w,
            Handler(Looper.getMainLooper()),
        ) {
            // Filament hands the rows back top first, already the right way up for a Bitmap.
            val bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
            pixels.rewind()
            bitmap.copyPixelsFromBuffer(pixels)
            done(bitmap)
        }
        renderer.readPixels(0, 0, w, h, descriptor)
    }

    fun destroy() {
        uiHelper.detach()
        swapChain?.let { engine.destroySwapChain(it) }
        swapChain = null
        loader.destroyAsset(asset)
        materials.destroyMaterials()
        materials.destroy()
        resources.destroy()
        loader.destroy()
        engine.destroyEntity(sun)
        engine.destroyIndirectLight(indirect)
        skybox?.let { engine.destroySkybox(it) }
        engine.destroyRenderer(renderer)
        engine.destroyView(view)
        engine.destroyScene(scene)
        engine.destroyCameraComponent(cameraEntity)
        EntityManager.get().destroy(cameraEntity)
        EntityManager.get().destroy(sun)
        engine.destroy()
    }

    companion object {
        init {
            Filament.init()
            Gltfio.init()
        }

        /** How far the beak opens at full voice. The real one opens about this much to quack. */
        const val MOUTH_OPEN = 0.35f

        private const val ELEVATION = 0.14f
        private val COS_ELEVATION = cos(ELEVATION)
        private val SIN_ELEVATION = sin(ELEVATION)

        /** A rotation about a unit axis, column-major, into [out]. */
        fun axisAngle(axis: FloatArray, angle: Float, out: FloatArray) {
            val c = cos(angle)
            val s = sin(angle)
            val t = 1 - c
            val x = axis[0]
            val y = axis[1]
            val z = axis[2]
            out[0] = t * x * x + c; out[1] = t * x * y + s * z; out[2] = t * x * z - s * y; out[3] = 0f
            out[4] = t * x * y - s * z; out[5] = t * y * y + c; out[6] = t * y * z + s * x; out[7] = 0f
            out[8] = t * x * z + s * y; out[9] = t * y * z - s * x; out[10] = t * z * z + c; out[11] = 0f
            out[12] = 0f; out[13] = 0f; out[14] = 0f; out[15] = 1f
        }

        /** A rotation (w x y z) and a translation, column-major, into [out]. */
        fun quatPosition(w: Float, x: Float, y: Float, z: Float, px: Float, py: Float, pz: Float, out: FloatArray) {
            out[0] = 1 - 2 * (y * y + z * z); out[1] = 2 * (x * y + w * z); out[2] = 2 * (x * z - w * y); out[3] = 0f
            out[4] = 2 * (x * y - w * z); out[5] = 1 - 2 * (x * x + z * z); out[6] = 2 * (y * z + w * x); out[7] = 0f
            out[8] = 2 * (x * z + w * y); out[9] = 2 * (y * z - w * x); out[10] = 1 - 2 * (x * x + y * y); out[11] = 0f
            out[12] = px; out[13] = py; out[14] = pz; out[15] = 1f
        }
    }
}
