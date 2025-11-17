package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class CachedAnalogInput extends CachedHardware {

    private final AnalogInput analogInput;
    private boolean updatedVoltage;
    private double voltage;
    public CachedAnalogInput(AnalogInput analogInput) {
        super(0); // caching tolerance doesn't mean anything for digital devices
        this.analogInput = analogInput;
        updatedVoltage = false;
        voltage = 0;
    }

    @Override
    public void resetCachedUpdates() {
        updatedVoltage = false;
    }

    @Override
    public void forceUpdateAllProperties() {
        resetCachedUpdates();
    }

    public double getVoltage() {
        if (!updatedVoltage) {
            voltage = analogInput.getVoltage();
            updatedVoltage = true;
        }
        return voltage;
    }
}
