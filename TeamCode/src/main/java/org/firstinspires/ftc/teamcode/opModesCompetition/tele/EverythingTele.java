package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.opModesTesting.PosePredictionErrorRecorder;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.BallColorSensor;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.OpmodeType;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;
import org.firstinspires.ftc.teamcode.utils.pinpoint.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.stateManagement.Subsystem;

@Config
public class EverythingTele extends ParentOpMode {
    public enum PosePredictType {
        SIMPLE,
        ADVANCED,
        CONTROL
    }
    public static PosePredictType posePredictType = PosePredictType.ADVANCED;
    public static double startX = 0, startY = 0, startA = 0;
    private Robot robot;
    public final Alliance alliance;

    private Pose2d lastFrameSimplePrediction, lastFrameAdvancedPrediction;

    private int framesRunning;
    private long startTime;
    public EverythingTele(Alliance alliance) {
        this.alliance = alliance;
    }
    @Override
    public void initiation() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardware, telemetry, OpmodeType.TELE, alliance, new Pose2d(startX, startY, startA));
        robot.declareHardware();
        robot.setInputInfo(new Keybinds(g1, g2));

        lastFrameSimplePrediction = new Pose2d(startX, startY, startA);
        lastFrameAdvancedPrediction = new Pose2d(startX, startY, startA);
        PosePredictionErrorRecorder.clearData();
        framesRunning = 0;
    }

    @Override
    public void start() {
        startTime = System.nanoTime();
    }

    @Override
    public void updateLoop() {
        double timeRunningSeconds = (System.nanoTime() - startTime) * 1.0 * 1e-9;
        robot.update();
        CommandScheduler.getInstance().run();

//        printRobotInfo();

        telemetry.addData("velocity list size", robot.pinpoint.previousVelocities.size());
        telemetry.addData("acceleration list size", robot.pinpoint.previousAccelerations.size());
        telemetry.addData("predicted dt", MathUtils.format(robot.pinpoint.getWeightedDt(), 5));
        telemetry.addData("last dt", MathUtils.format(robot.pinpoint.previousDeltaTimes.get(0), 5));
        telemetry.addData("time running", MathUtils.format3(timeRunningSeconds));
        telemetry.addData("average FPS", MathUtils.format3(framesRunning / timeRunningSeconds));
        Pose2d actualPose = robot.pinpoint.pose();
        switch (posePredictType) {
            case SIMPLE: TelemetryHelper.sendRobotPose(actualPose, lastFrameSimplePrediction); break;
            case ADVANCED: TelemetryHelper.sendRobotPose(actualPose, lastFrameAdvancedPrediction); break;
            case CONTROL: TelemetryHelper.sendRobotPose(actualPose, robot.pinpoint.lastPose); break;
        }
        if (keybinds.check(Keybinds.D1Trigger.TRACK_POSE_PREDICT))
            trackPosePredict(actualPose);
        lastFrameSimplePrediction = robot.pinpoint.getNextPoseSimple();
        lastFrameAdvancedPrediction = robot.pinpoint.getNextPoseAdvanced();

        telemetry.update();
        framesRunning++;
    }

    private void printRobotInfo() {
        telemetry.addData("alliance", robot.alliance);
        telemetry.addLine("=====SUBSYSTEMS=====");
        for (Subsystem subsystem : robot.subsystems) {
            subsystem.printInfo();
            telemetry.addLine();
        }

        telemetry.addLine("=====SENSORS=====");
        robot.pinpoint.printInfo();
        telemetry.addLine();
        for (BallColorSensor colorSensor : robot.colorSensors)
            colorSensor.printInfo();
    }

    private void trackPosePredict(Pose2d actualPose) {
        // save simple and advanced errors
        OdoInfo simpleError = new OdoInfo(lastFrameSimplePrediction.position.x - actualPose.position.x,
                lastFrameSimplePrediction.position.y - actualPose.position.y,
                HeadingCorrect.correctHeadingErrorRad(lastFrameSimplePrediction.heading.toDouble() - actualPose.heading.toDouble()));
        PosePredictionErrorRecorder.predictionErrorsSimple.add(simpleError);
        OdoInfo advancedError = new OdoInfo(lastFrameAdvancedPrediction.position.x - actualPose.position.x,
                lastFrameAdvancedPrediction.position.y - actualPose.position.y,
                HeadingCorrect.correctHeadingErrorRad(lastFrameAdvancedPrediction.heading.toDouble() - actualPose.heading.toDouble()));
        PosePredictionErrorRecorder.predictionErrorsAdvanced.add(advancedError);

        // save current velocity and acceleration
        if (!robot.pinpoint.previousAccelerations.isEmpty())
            PosePredictionErrorRecorder.acceleration.add(robot.pinpoint.previousAccelerations.get(0));
        if (!robot.pinpoint.previousVelocities.isEmpty())
            PosePredictionErrorRecorder.velocity.add(robot.pinpoint.previousVelocities.get(0));

        // save control group errors
        Pose2d lastPose = robot.pinpoint.lastPose;
        OdoInfo controlGroupError = new OdoInfo(lastPose.position.x - actualPose.position.x,
                lastPose.position.y - actualPose.position.y,
                lastPose.heading.toDouble() - actualPose.heading.toDouble()
        );
        PosePredictionErrorRecorder.controlGroupError.add(controlGroupError);
    }
}
