package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class InterpolatingShotTreeMapContainer {

    public static final InterpolatingShotTreeMap shotMap = new InterpolatingShotTreeMap();
    double[][] values = {
        // TODO: find these values
        { 1.1, 14.3, 3000 }, //Same as last to prevent NPE
        { 1.5, 13.6, 3000 }, //30, 14.25, 2000
        { 2.3, 12.9, 3500 }, //54, 13.75, 2100
        { 2.9, 12.73, 3500 },
        { 139, 12.65, 2900 },
        { 170, 12.5, 3200 },
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
