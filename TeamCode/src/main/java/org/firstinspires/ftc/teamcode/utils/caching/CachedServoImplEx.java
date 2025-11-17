package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class CachedServoImplEx extends CachedHardware {
    public static double defaultCachingTolerance = 0;
    private final ServoImplEx servo;
    private double position;
    private boolean updatedPosition;
    public CachedServoImplEx(ServoImplEx servo) {
        this(servo, defaultCachingTolerance);
    }
    public CachedServoImplEx(ServoImplEx servo, double cachingTolerance) {
        super(cachingTolerance);
        this.servo = servo;
        position = servo.getPosition();
    }

    @Override
    public void resetCachedUpdates() {
        updatedPosition = false;
    }

    @Override
    public void forceUpdateAllProperties() {
        resetCachedUpdates();
        getPosition();
    }

    public double getPosition() {
        if (!updatedPosition) {
            position = servo.getPosition();
            updatedPosition = true;
        }
        return position;
    }
    public void setPosition(double position) {
        if (Math.abs(position - this.position) > getCachingTolerance()) {
            this.position = position;
            servo.setPosition(position);
        }
    }
    public ServoImplEx getServo() {
        return servo;
    }
}
