package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opModesTesting.ShooterSpeedRecorder;
import org.firstinspires.ftc.teamcode.opModesCompetition.tele.Keybinds;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleCommand;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDFController;
import org.firstinspires.ftc.teamcode.utils.math.CubicEquation;
import org.firstinspires.ftc.teamcode.utils.math.QuadraticEquation;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class Shooter extends Subsystem {
    public static boolean ENABLE_TESTING = false;
    public static Zone defaultZone = Zone.FAR;

    public void setCustomHoodOffset(double hoodOffset) {
        this.customHoodOffset = hoodOffset;
    }
    private double customHoodOffset = 0;

    public static class ShootNearParams {
        public double minPower = -0.2;
        public double maxPowerSpeedErrorThreshold = 10;
        public double shooterKp = 0.1, shooterKi = 0, shooterKd = 0, shooterKf = 0.2;
        public double targetSpeed = 360, targetConstantWeighting = 0.7, targetConstant = 0.94;
        public double maxSpeed = 375, minSpeed = 355, extraCloseMinSpeed = 330;
        public double maxSpeedUpTime = 7;
    }
    public static class ShootFarParams {
        public double shooterKp = 0.1, shooterKi = 0, shooterKd = 0, shooterKf = 0.4;
        public double targetSpeed = 480, targetConstantWeighting = 0.7, targetConstant = 0.96;
        public double maxSpeed = 500, extraCloseMinSpeed = 460, minSpeed = 470;
    }

    public static class HoodParams {
        public double downPosition = 0.76, upPosition = 0.3;
        public double defaultPosition = 0.7;
        public double shootParkOffset = -0.02;
        public double manualChangeAmount = 0.01;
        public CubicEquation nearZoneHoodEquation = new CubicEquation(0.000007177106, -0.007655489, 2.715645, -319.70495);
        public QuadraticEquation farZoneHoodEquation = new QuadraticEquation(0.000113087, -0.105996, 25.40292);
    }
    public static ShootNearParams nearParams = new ShootNearParams();
    public static ShootFarParams farParams = new ShootFarParams();
    public static HoodParams hoodParams = new HoodParams();
    public static long maxShootTimeMs = 5000;
    public static double passivePower = 0.35, intakeOnPower = 0.7;

    public enum State {
        OFF,
        TRACK_PASSIVE_SPEED,
        TRACK_SHOOTER_SPEED
    }
    public enum Zone {
        NEAR,
        FAR
    }
    private State state;
    private Zone zone;
    private DcMotorEx leftMotor, rightMotor;
    private ServoImplEx leftServo, rightServo;
    private final PIDFController speedPidf; // input error between current speed and target speed, output desired power
    private double targetHoodPos, pidMotorPower;
    private final ElapsedTime recordTimer;
    private double targetSpeed;
    private final ElapsedTime stateTimer;
    private boolean shooting = false;
    public Shooter(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);
        stateTimer = new ElapsedTime();
        stateTimer.reset();
        recordTimer = new ElapsedTime();

        zone = defaultZone;
        targetSpeed = getTargetMotorSpeed();
        speedPidf = new PIDFController(0, 0, 0, 0);
        resetSpeedPIDToCurrentZone();
        speedPidf.setOutputBounds(0, 0.99);
        state = State.TRACK_PASSIVE_SPEED;
        ShooterSpeedRecorder.resetData();
    }

    @Override
    public void declareHardware() {
        leftMotor = hardware.getLeftShooterMotor();
        rightMotor = hardware.getRightShooterMotor();
        leftServo = hardware.getLeftHoodServo();
        rightServo = hardware.getRightHoodServo();
        targetHoodPos = hoodParams.defaultPosition;
        setRawServoPositions(targetHoodPos);
    }

    @Override
    public void updateState() {
        if (ENABLE_TESTING) {
            if (keybinds.g1.isDpadUpPressed()) {
                targetHoodPos -= hoodParams.manualChangeAmount;
                setRawServoPositions(targetHoodPos);
            } else if (keybinds.g1.isDpadDownPressed()) {
                targetHoodPos += hoodParams.manualChangeAmount;
                setRawServoPositions(targetHoodPos);
            }
        }

        if (keybinds.check(Keybinds.D2Trigger.SHOOT_NEAR))
            setZone(Zone.NEAR);
        else if (keybinds.check(Keybinds.D2Trigger.SHOOT_FAR))
            setZone(Zone.FAR);

        switch (state) {
            case OFF:
                if (keybinds.check(Keybinds.D1Trigger.TRACK_SHOOTER_SPEED)) {
                    setState(State.TRACK_SHOOTER_SPEED);
                    break;
                }
                if (keybinds.check(Keybinds.D2Trigger.INCREASE_SHOOTER_SPEED_STATE)) {
                    setState(State.TRACK_PASSIVE_SPEED);
                    break;
                }
                break;
            case TRACK_PASSIVE_SPEED:
                if (keybinds.check(Keybinds.D1Trigger.TRACK_SHOOTER_SPEED) || keybinds.check(Keybinds.D2Trigger.INCREASE_SHOOTER_SPEED_STATE)) {
                    setState(State.TRACK_SHOOTER_SPEED);
                    break;
                }
                if (keybinds.check(Keybinds.D2Trigger.DECREASE_SHOOTER_SPEED_STATE)) {
                    setState(State.OFF);
                    break;
                }
                if (!ENABLE_TESTING)
                    setMotorPowers(passivePower);
                break;
            case TRACK_SHOOTER_SPEED:
                if (keybinds.check(Keybinds.D2Trigger.DECREASE_SHOOTER_SPEED_STATE)) {
                    setState(State.TRACK_PASSIVE_SPEED);
                    break;
                }

                if (keybinds.check(Keybinds.D1Trigger.START_SHOOTING) && robot != null)
                    robot.shootBallCommand(false, false).schedule();


                if (!ENABLE_TESTING) {
                    if (zone == Zone.NEAR)
                        targetHoodPos = hoodParams.nearZoneHoodEquation.calculate(getAvgMotorSpeed());
                    else
                        targetHoodPos = hoodParams.farZoneHoodEquation.calculate(getAvgMotorSpeed());

                    if (robot.park.isParkedForShoot())
                        targetHoodPos += hoodParams.shootParkOffset;
                    targetHoodPos += customHoodOffset;
                    setRawServoPositions(targetHoodPos);
                }

                // quicker speed up
                if (getAvgMotorSpeed() < getTargetMotorSpeed() - nearParams.maxPowerSpeedErrorThreshold)
                    setMotorPowers(0.99);
                else {
                    pidMotorPower = speedPidf.update(getAvgMotorSpeed());

                    // weight pid power to reduce variance
                    pidMotorPower = MathUtils.lerp(pidMotorPower, getTargetSpeedConstant(), getTargetSpeedConstantWeighting());
                    pidMotorPower = Math.max(nearParams.minPower, pidMotorPower);

                    if (robot.intake.getState() == Intake.State.ON)
                        setMotorPowers(Math.min(pidMotorPower, intakeOnPower)); // give collector some lenience
                    else
                        setMotorPowers(pidMotorPower);
                }
                break;
        }
    }

    public void setState(State newState) {
        if (state == newState)
            return;

        state = newState;
        stateTimer.reset();

        if (state == State.OFF) {
            setMotorPowers(0);
            setRawServoPositions(hoodParams.downPosition);
        }
        else if (state == State.TRACK_PASSIVE_SPEED) {
            if (ENABLE_TESTING)
                setMotorPowers(0);
            else
                setMotorPowers(passivePower);
        }
        else if (state == State.TRACK_SHOOTER_SPEED) {
            speedPidf.reset();
            speedPidf.setTarget(targetSpeed);
            if (robot.intake.getState() == Intake.State.OFF)
                robot.intake.setState(Intake.State.PASSIVE_INTAKE);
        }
    }
    private void setMotorPowers(double power) {
        if (power < nearParams.minPower)
            throw new IllegalArgumentException("shooter power of " + power + " cannot be less than min power of " + nearParams.minPower);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public double getAvgMotorSpeed() {
        double leftSpeed = Math.abs(leftMotor.getVelocity(AngleUnit.DEGREES));
        double rightSpeed = Math.abs(rightMotor.getVelocity(AngleUnit.DEGREES));
        return (leftSpeed + rightSpeed) / 2;
    }
    public double getAvgServoPosition() {
        return (leftServo.getPosition() + rightServo.getPosition()) / 2.;
    }
    private void setRawServoPositions(double pos) {
        pos = Range.clip(pos, hoodParams.upPosition, hoodParams.downPosition);
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    public boolean canShootThreeNear() {
        return getAvgMotorSpeed() > nearParams.minSpeed;
    }
    public boolean canShootThreeFar() {
        return getAvgMotorSpeed() > farParams.minSpeed;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===SHOOTER===");
        telemetry.addData("state", state);
        telemetry.addData("zone", zone);
        telemetry.addLine();
        telemetry.addData("pid motor target", speedPidf.getTarget());
        telemetry.addData("pid motor power", pidMotorPower);
        telemetry.addData("actual avg speed", MathUtils.format3(getAvgMotorSpeed()));
        telemetry.addData("left speed", leftMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right speed", rightMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("left motor power", MathUtils.format3(leftMotor.getPower()));
        telemetry.addData("right motor power", MathUtils.format3(rightMotor.getPower()));
        telemetry.addData("left servo position", MathUtils.format3(leftServo.getPosition()));
        telemetry.addData("right servo position", MathUtils.format3(rightServo.getPosition()));
    }

    public double getAvgShooterPower() {
        return (leftMotor.getPower() + rightMotor.getPower()) * 0.5;
    }
    public Command shooterTrackerCommand() {
        return new SimpleCommand() {
            private int num = 0;
            private double lastTime = 0;
            @Override
            public void run() {
                if (recordTimer.milliseconds() - lastTime > ShooterSpeedRecorder.recordIntervalMs
                        && num < ShooterSpeedRecorder.recordAmountForEachShot
                        && ShooterSpeedRecorder.getCurrentShot() < ShooterSpeedRecorder.numShotsToRecord) {
                    lastTime = recordTimer.milliseconds();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][0] = recordTimer.seconds();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][1] = getTargetMotorSpeed();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][2] = getAvgMotorSpeed();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][3] = getAvgShooterPower();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][4] = getAvgServoPosition();
                    num++;
                }
            }
            @Override
            public void end(boolean interrupted) {
                ShooterSpeedRecorder.incrementCurrentShot();
            }
            @Override
            public boolean isDone() {
                boolean intakeDone = robot.intake.getState() != Intake.State.PASSIVE_INTAKE && robot.intake.getState() != Intake.State.FEED_SHOOTER_TELE_TOGGLE && robot.intake.getState() != Intake.State.FEED_SHOOTER_PRECISE;
                return (intakeDone && recordTimer.seconds() >= 0.3) || num >= ShooterSpeedRecorder.recordAmountForEachShot;
            }
        };
    }

    private double getTargetMotorSpeed() {
        return zone == Zone.NEAR ? nearParams.targetSpeed : farParams.targetSpeed;
    }
    private double getTargetSpeedConstantWeighting() {
        return zone == Zone.NEAR ? nearParams.targetConstantWeighting : farParams.targetConstantWeighting;
    }
    private double getTargetSpeedConstant() {
        return zone == Zone.NEAR ? nearParams.targetConstant : farParams.targetConstant;
    }
    public Zone getZone() {
         return zone;
    }
    public void setZone(Zone zone) {
        this.zone = zone;
        resetSpeedPIDToCurrentZone();
        setTargetSpeed(zone == Zone.NEAR ? nearParams.targetSpeed : farParams.targetSpeed);
    }

    public boolean canShoot() {
        return zone == Zone.NEAR ? canShootThreeNear() : canShootThreeFar();
    }
    public boolean canShootExtraNear() {
        return getAvgMotorSpeed() > nearParams.extraCloseMinSpeed;
    }

    public void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
        speedPidf.setTarget(targetSpeed);
        speedPidf.reset();
    }

    public State getState() {
        return state;
    }
    public double getStateTime() {
        return stateTimer.seconds();
    }
    public void resetSpeedPIDToCurrentZone() {
        if (zone == Zone.NEAR) {
            speedPidf.setPIDValues(nearParams.shooterKp, nearParams.shooterKi, nearParams.shooterKd, nearParams.shooterKf);
        }
        else {
            speedPidf.setPIDValues(farParams.shooterKp, farParams.shooterKi, farParams.shooterKd, farParams.shooterKf);
        }
    }
    public boolean isShooting() {
        return shooting;
    }

    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }
}
