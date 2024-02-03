package frc.robot.util;

import java.util.TreeMap;

public class InterpolatingShotTreeMap {

    // Shot tree map
    private final TreeMap<Double, InterpolatableShotData> STM = new TreeMap<>();

    public InterpolatingShotTreeMap() {
    }

    public void put(Double distance, InterpolatableShotData value) {
        STM.put(distance, value);
    }

    public InterpolatableShotData get(Double distance) {
        InterpolatableShotData val = STM.get(distance);
        if (val == null) {
            Double ceilingKey = STM.ceilingKey(distance);
            Double floorKey = STM.floorKey(distance);

            if (ceilingKey == null || floorKey == null) {
                return null;
            }

            InterpolatableShotData floor = STM.get(floorKey);
            InterpolatableShotData ceiling = STM.get(ceilingKey);

            return interpolate(floor, ceiling, inverseInterpolate(ceilingKey, distance, floorKey));
        } else {
            return val;
        }
    }

    private InterpolatableShotData interpolate(
            InterpolatableShotData startValue,
            InterpolatableShotData endValue,
            double t) {
        return new InterpolatableShotData(
                ((endValue.getArmAngle() - startValue.getArmAngle()) * t) + startValue.getArmAngle(),
                ((endValue.getRPM() - startValue.getRPM()) * t) + startValue.getRPM());
    }

    private double inverseInterpolate(Double up, Double q, Double down) {
        double upperToLower = up.doubleValue() - down.doubleValue();
        if (upperToLower <= 0) {
            return 0.0;
        }
        double queryToLower = q.doubleValue() - down.doubleValue();
        if (queryToLower <= 0) {
            return 0.0;
        }
        return queryToLower / upperToLower;
    }
}
// https://github.com/HighlanderRobotics/Crescendo/tree/main/src/main/java/frc/robot/utils/autoaim
