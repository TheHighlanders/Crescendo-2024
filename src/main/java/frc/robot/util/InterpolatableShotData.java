package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolatable;

public class InterpolatableShotData implements Interpolatable<InterpolatableShotData> {

    private double armExtension;
    private double rotationsPerSecond;

    public InterpolatableShotData(double armAngle, double rotationsPerSecond) {
        this.armExtension = armAngle;
        this.rotationsPerSecond = rotationsPerSecond;
    }

    public double getArmExtension() {
        return armExtension;
    }

    public double getRPM() {
        return rotationsPerSecond;
    }

    @Override
    public InterpolatableShotData interpolate(InterpolatableShotData endValue, double t) {
        // Unused actual code in ISTM
        return new InterpolatableShotData(
            ((endValue.getArmExtension() - armExtension) * t) + armExtension,
            ((endValue.getRPM() - rotationsPerSecond) * t) + rotationsPerSecond
        );
    }
}
