package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolatable;

public class InterpolatableShotData implements Interpolatable<InterpolatableShotData> {

    private double armAngle;
    private double rotationsPerSecond;

    public InterpolatableShotData(double armAngle, double rotationsPerSecond) {
        this.armAngle = armAngle;
        this.rotationsPerSecond = rotationsPerSecond;
    }

    public double getArmAngle() {
        return armAngle;
    }

    public double getRPM() {
        return rotationsPerSecond;
    }

    @Override
    public InterpolatableShotData interpolate(InterpolatableShotData endValue, double t) {
        return new InterpolatableShotData(
                ((endValue.getArmAngle() - armAngle) * t) + armAngle,
                ((endValue.getRPM() - rotationsPerSecond) * t) + rotationsPerSecond);
    }
}
