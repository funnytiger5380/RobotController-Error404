package org.firstinspires.ftc.teamcode.pedroPathing

import com.bylazar.field.CanvasRotation
import com.bylazar.field.FieldPluginConfig
import com.bylazar.field.FieldPresetParams

class FieldConfig : FieldPluginConfig() {
    override var extraPresets: List<FieldPresetParams> = listOf(
        FieldPresetParams(
            name = "Customized",
            offsetX = -24.0 * 5,
            offsetY = -24.0 * 5,
            flipX = false,
            flipY = true,
            reverseXY = false,
            rotation = CanvasRotation.DEG_90))
}