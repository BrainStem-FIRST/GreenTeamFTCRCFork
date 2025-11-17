package org.firstinspires.ftc.teamcode.utils.caching;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedDcMotorEx extends CachedHardware {
    public static double defaultCachingTolerance = 0;
    private final DcMotorEx motor;
    private int position;
    private double velocity, current;
    private boolean updatedPosition, updatedVelocity, updatedCurrent;
    private double power;

    public CachedDcMotorEx(DcMotorEx motor) {
        this(motor, defaultCachingTolerance);
    }
    public CachedDcMotorEx(DcMotorEx motor, double cachingTolerance) {
        super(cachingTolerance);
        this.motor = motor;
        power = 0;
        position = 0;
        velocity = 0;
        current = 0;
        updatedPosition = false;
        updatedVelocity = false;
        updatedCurrent = false;
    }

    @Override
    public void resetCachedUpdates() {
        updatedPosition = false;
        updatedVelocity = false;
        updatedCurrent = false;
    }
    @Override
    public void forceUpdateAllProperties() {
        resetCachedUpdates();
        getCurrentPosition();
        getVelocity();
        getCurrent();
    }

    public int getCurrentPosition() {
        if (!updatedPosition) {
            position = motor.getCurrentPosition();
            updatedPosition = true;
        }
        return position;
    }
    public double getVelocity() {
        if (!updatedVelocity) {
            velocity = motor.getVelocity();
            updatedVelocity = true;
        }
        return velocity;
    }
    public double getCurrent() {
        if (!updatedCurrent) {
            current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            updatedCurrent = true;
        }
        return current;
    }
    public double getPower() {
        return power;
    }
    public void setPower(double power) {
        if (Math.abs(power - this.power) > getCachingTolerance()) {
            motor.setPower(power);
            this.power = power;
        }
    }

    public DcMotorEx getMotor() {
        return motor;
    }
}
