package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class TelemetryHelper {
    public static String[] colors = { "red", "green" };
    public static Integer[] strokeWidths = { 1, 1 };
    public static double strokeAlpha = 0.5;
    public static double fieldRotation = 90, robotRadius = 5;

    public static void sendRobotPose(Pose2d ...poses) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setRotation(Math.toRadians(fieldRotation)); // rotate 90deg clockwise

        for (int i=0; i<poses.length; i++) {
            Pose2d pose = poses[i];
            fieldOverlay.setAlpha(strokeAlpha);
            fieldOverlay.setStrokeWidth(strokeWidths[i]);
            fieldOverlay.setStroke(colors[i % colors.length]);
            double x = pose.position.x, y = pose.position.y, heading = pose.heading.toDouble();
            fieldOverlay.strokeCircle(x, y, robotRadius);
            fieldOverlay.strokeLine(x, y, x + robotRadius * Math.cos(heading), y + robotRadius * Math.sin(heading));
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
