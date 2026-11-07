package io.github.samimreza.virtualduck.motion

import org.json.JSONArray
import org.json.JSONObject

/**
 * Which node of the model each joint turns, about which axis and within which range — and the
 * beak's hinge, which the MJCF does not have and `export_model.py` works out from the jaw's shape.
 * Written beside the model as `duck_rig.json`.
 */
class Rig(
    val trunk: String,
    val joints: List<Joint>,
    val mouthParts: List<String>,
    /** In the head's frame. */
    val mouthPivot: Vec3,
    val mouthAxis: Vec3,
) {
    class Joint(val name: String, val node: String, val axis: Vec3, val min: Float, val max: Float)

    companion object {
        fun parse(text: String): Rig {
            val root = JSONObject(text)
            val joints = root.getJSONArray("joints").let { a ->
                List(a.length()) {
                    val j = a.getJSONObject(it)
                    val range = j.getJSONArray("range")
                    Joint(
                        j.getString("name"),
                        j.getString("node"),
                        vec(j.getJSONArray("axis")),
                        range.getDouble(0).toFloat(),
                        range.getDouble(1).toFloat(),
                    )
                }
            }
            val mouth = root.getJSONObject("mouth")
            return Rig(
                root.getString("trunk"),
                joints,
                mouth.getJSONArray("parts").let { a -> List(a.length()) { a.getString(it) } },
                vec(mouth.getJSONArray("pivot")),
                vec(mouth.getJSONArray("axis")),
            )
        }

        private fun vec(a: JSONArray) = Vec3(a.getDouble(0).toFloat(), a.getDouble(1).toFloat(), a.getDouble(2).toFloat())
    }
}
