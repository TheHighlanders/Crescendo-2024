package frc.robot.util;

public class InterpolatingShotTreeMapContainer {

    public static final InterpolatingShotTreeMap shotMap = new InterpolatingShotTreeMap();
    double[][] values = {
        // TODO: find these values
        { 1.1, 14.3, 3000 }, //Same as last to prevent NPE
        { 1.5, 13.8, 3000 }, //30, 14.25, 2000
        { 2.3, 13.0 + 0.125, 3000 }, //54, 13.75, 2100 // distance, extenstion,
        { 3.7, 12.75 + 0.125, 3000 },
        { 5.93, 12.4, 3500 },
    };

    public InterpolatingShotTreeMapContainer() {
        for (int i = 0; i < values.length; i++) {
            double distance = values[i][0];
            double extension = values[i][1];
            double rpm = values[i][2];

            InterpolatableShotData shotData = new InterpolatableShotData(extension, rpm);

            shotMap.put(distance, shotData);
        }
    }

    public InterpolatableShotData interpolate(double dist) {
        return shotMap.get(dist);
    }
}
