package frc.robot.util;

public class InterpolatingShotTreeMapContainer {

    public static final InterpolatingShotTreeMap shotMap = new InterpolatingShotTreeMap();
    double[][] values = {
        // TODO: find these values
        { 10.0, 20.0, 30.0, 40.0, 50.0 }, // Distances
        { 45.0, 30.0, 60.0, 45.0, 75.0 }, // Extension
        { 1000.0, 1200.0, 900.0, 1100.0, 800.0 }, // RPM's
    };

    public InterpolatingShotTreeMapContainer() {
        for (int i = 0; i < values[0].length; i++) {
            double distance = values[0][i];
            double extension = values[1][i];
            double rpm = values[2][i];

            double rps = rpm / 60.0;

            InterpolatableShotData shotData = new InterpolatableShotData(extension, rps);

            shotMap.put(distance, shotData);
        }
    }

    public InterpolatableShotData interpolate(double dist) {
        return shotMap.get(dist);
    }
}
