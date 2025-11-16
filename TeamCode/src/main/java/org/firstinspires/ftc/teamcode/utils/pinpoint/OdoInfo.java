package org.firstinspires.ftc.teamcode.utils.pinpoint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

public class OdoInfo {
    public double x, y, headingRad;
    public OdoInfo(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }
    public OdoInfo() {
        x = 0;
        y = 0;
        headingRad = 0;
    }
    @NonNull
    @Override
    public OdoInfo clone() {
        return new OdoInfo(x, y, headingRad);
    }

    public String toString(int numDecimalPlaces) {
        return "x:" + MathUtils.format(x, numDecimalPlaces) + "y:" + MathUtils.format(y, numDecimalPlaces) + "h:" + MathUtils.format(headingRad, numDecimalPlaces);
    }
    public String toStringPosition(int numDecimalPlaces) {
        return "x:" + MathUtils.format(x, numDecimalPlaces) + "y:" + MathUtils.format(y, numDecimalPlaces);
    }
    public String toStringHeading(int numDecimalPlaces) {
        return "h:" + MathUtils.format(headingRad, numDecimalPlaces);

    }
}
