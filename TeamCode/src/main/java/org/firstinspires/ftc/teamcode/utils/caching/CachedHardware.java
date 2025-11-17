package org.firstinspires.ftc.teamcode.utils.caching;

public abstract class CachedHardware {
    private double cachingTolerance;

    public CachedHardware(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
    }

    public double getCachingTolerance() {
        return cachingTolerance;
    }
    public void setCachingTolerance(double tolerance) {
        this.cachingTolerance = tolerance;
    }

    public abstract void resetCachedUpdates();
    public abstract void forceUpdateAllProperties();
}
