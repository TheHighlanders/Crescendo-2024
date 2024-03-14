package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolatable;

public class InterpolatableShotData implements Interpolatable<InterpolatableShotData> {

    private double armExtension;
    private double rpm;

    public InterpolatableShotData(double armAngle, double rpm) {
        this.armExtension = armAngle;
        this.rpm = rpm;
    }

    public double getArmExtension() {
        return armExtension;
    }

    public double getRPM() {
        return rpm;
    }

    @Override
    public InterpolatableShotData interpolate(InterpolatableShotData endValue, double t) {
        // Unused actual code in ISTM
        return new InterpolatableShotData(((endValue.getArmExtension() - armExtension) * t) + armExtension, ((endValue.getRPM() - rpm) * t) + rpm);
    }
}
