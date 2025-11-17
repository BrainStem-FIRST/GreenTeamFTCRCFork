package org.firstinspires.ftc.teamcode.opModesCompetition.loopTimeTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

import java.util.List;

/*
questions i am trying to answer:
how long does each getPower call take?
how long does each setPower call take?
how much time does bulk caching save?
what are the highest possible loop times i can achieve?
does bulk caching ignore redundant setPower calls?
 */

@TeleOp(name="Measuring Loop Times", group="Loop Times")
@Config
public class MeasuringLoopTimes extends OpMode {
    // hyper parameters
    public static String motorName = "intake";
    public static int numGetPowerCallsPerLoop = 0, numSetPowerCallsPerLoop = 0;
    public static boolean useBulkCaching = false;
    public static boolean changeMotorPower = false;
    public static int telemetryMsTransmissionInterval = 20;

    // instance data
    private int numLoops;
    private double totalDeltaTimeMs;
    private DcMotorEx motor;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(telemetryMsTransmissionInterval);
        numLoops = 0;
        totalDeltaTimeMs = 0;
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        allHubs = hardwareMap.getAll(LynxModule.class);
        // set bulk reading mode for each hub
        if (useBulkCaching)
            for (LynxModule hub : allHubs)
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void loop() {
        // clears cache on each hub
        if (useBulkCaching)
            for (LynxModule hub : allHubs)
                hub.clearBulkCache();

        // perform hardware calls
        long startTime = System.nanoTime();
        for (int i=0; i<numGetPowerCallsPerLoop; i++)
            motor.getPower();
        for (int i=0; i<numSetPowerCallsPerLoop; i++)
            motor.setPower(changeMotorPower ? 0.5 * Math.sin(time * 0.25) : 0.2);

        // track loop times
        double deltaTimeMs = (System.nanoTime() - startTime) * 1.0e-6;
        totalDeltaTimeMs += deltaTimeMs;

        numLoops++;

        telemetry.addData("delta time", MathUtils.format3(deltaTimeMs));
        telemetry.addData("average delta time", MathUtils.format3(totalDeltaTimeMs / numLoops));
        telemetry.addData("average loop time (ms)", MathUtils.format3(1000 * numLoops / time));

        telemetry.update();
    }
}
