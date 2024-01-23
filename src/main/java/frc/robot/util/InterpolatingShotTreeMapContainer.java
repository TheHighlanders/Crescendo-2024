package frc.robot.util;

public class InterpolatingShotTreeMapContainer {
    public static final InterpolatingShotTreeMap shotMap = new InterpolatingShotTreeMap();
    double[][] values = {
            {10.0, 20.0, 30.0, 40.0, 50.0}, // Distances
            {45.0, 30.0, 60.0, 45.0, 75.0}, // Angles
            {1000.0, 1200.0, 900.0, 1100.0, 800.0} // RPMs
        };

    InterpolatingShotTreeMapContainer() {
        for (int i = 0; i < values[0].length; i++) {
            double distance = values[0][i];
            double angle = values[1][i];
            double rpm = values[2][i];

            double rps = rpm / 60.0;

            InterpolatableShotData shotData = new InterpolatableShotData(angle, rps);

            shotMap.put(distance, shotData);
        }
    }

    public InterpolatableShotData interpolate(double dist) {
        return shotMap.get(dist);
    }
}
