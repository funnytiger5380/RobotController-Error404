package org.firstinspires.ftc.teamcode.pedroPathing

import com.bylazar.field.CanvasRotation
import com.bylazar.field.FieldPluginConfig
import com.bylazar.field.FieldPresetParams

class FieldConfig : FieldPluginConfig() {
    override var extraPresets: List<FieldPresetParams> = listOf(
        FieldPresetParams(
            name = "Pedro DEG_180",
            offsetX = -24.0 * 3,
            offsetY = -24.0 * 3,
            flipX = false,
            flipY = true,
            reverseXY = false,
            rotation = CanvasRotation.DEG_180))
}