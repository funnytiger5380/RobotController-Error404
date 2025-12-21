package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DigitalSensor {
    private DigitalChannel sensor;
    private DigitalChannel.Mode mode = DigitalChannel.Mode.INPUT;
    private String sensorName = "digitalSensor";

    public DigitalSensor sensorName(String sensorName) {
        this.sensorName = sensorName;
        return this;
    }

    public DigitalSensor sensorMode(DigitalChannel.Mode mode) {
        this.mode = mode;
        return this;
    }

    public void build(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(DigitalChannel.class, sensorName);
        sensor.setMode(mode);
    }

    public boolean isDetected() {
        return sensor.getState();
    }
}
