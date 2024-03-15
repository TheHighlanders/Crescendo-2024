package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class InterpolatingShotTreeMapContainer {

    public static final InterpolatingShotTreeMap shotMap = new InterpolatingShotTreeMap();
    double[][] values = {
        // TODO: find these values
        { 36, 14.25, 2000 },
        { 60, 13.75, 2100 },
        { 108, 12.9, 2500 },
        { 145, 12.65, 2900 },
        { 176, 12.5, 3200 },
    };

    public InterpolatingShotTreeMapContainer() {
        for (int i = 0; i < values.length; i++) {
            double distance = Units.inchesToMeters(values[i][0]);
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
