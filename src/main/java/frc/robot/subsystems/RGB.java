// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.EnumMap;

public class RGB extends SubsystemBase {

    /** Creates a new RGB. */
    private SerialPort elPuerto;

    enum State {
        TEST,
        OFF,
        RED,
        BLUE,
        RAINBOW,
        ORANGEBLINK,
        ORANGESOLID,
    }

    public EnumMap<State, String> stateMap = new EnumMap<>(State.class);

    public RGB() {
        elPuerto = new SerialPort(9600, SerialPort.Port.kMXP);
        stateMap.put(State.TEST, "0");
        stateMap.put(State.OFF, "1");
        stateMap.put(State.RED, "2");
        stateMap.put(State.BLUE, "3");
        stateMap.put(State.RAINBOW, "4");
        stateMap.put(State.ORANGEBLINK, "5");
        stateMap.put(State.ORANGESOLID, "6");

        // setLED(State.RAINBOW);
        setArmLEDLoadingBar(99);
    }

    public void changeString(String str) {
        elPuerto.writeString(str + "\n");
        DriverStation.reportWarning(str, false);
    }

    public void setLED(State state) {
        changeString(stateMap.get(state));
    }

    /**
     *
     * @param angleDif Angle Difference between arm and pivot !DEG! 100DEG is max
     */
    public void setArmLEDLoadingBar(double angleDif) {
        angleDif = Math.max(Math.min(Math.abs(angleDif), 100), 0);

        int state = ((int) angleDif) + 100;
        changeString(state + "");
    }

    @Override
    public void periodic() {
        // DriverStation.reportWarning("something", false);
        // This method will be called once per scheduler run
    }
}
