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

    public enum State {
        TEST,
        OFF,
        RED,
        BLUE,
        RAINBOW,
        ORANGEBLINK,
        ORANGESOLID,
        POPSICLE,
        BROWN,
        FLAMEGRADIENT,
        FLAME,
        LOADINGBAR,
        PARTYPOOPMODE
    }

    private boolean poopMode;

    public EnumMap<State, String> stateMap = new EnumMap<>(State.class);

    public RGB() {
        elPuerto = new SerialPort(9600, SerialPort.Port.kUSB);
        stateMap.put(State.TEST, "0");
        stateMap.put(State.OFF, "1");
        stateMap.put(State.RED, "2");
        stateMap.put(State.BLUE, "3");
        stateMap.put(State.RAINBOW, "4");
        stateMap.put(State.ORANGEBLINK, "5");
        stateMap.put(State.ORANGESOLID, "6");
        stateMap.put(State.POPSICLE, "7");
        stateMap.put(State.BROWN, "8");
        stateMap.put(State.LOADINGBAR, "9");
        stateMap.put(State.FLAMEGRADIENT, "10");
        stateMap.put(State.FLAME, "11");
        stateMap.put(State.PARTYPOOPMODE, "12");

        setLED(State.OFF);
        // setArmLEDLoadingBar(10, 30);
        poopMode = false;
    }

    public void changeString(String str) {
        try {
            elPuerto.writeString(str + "\n");
        } catch (Throwable e) {
            DriverStation.reportWarning("AHHH",true);
        }
    }

    public void setLED(State state) {
        if (!poopMode) {
            changeString(stateMap.get(state));
        }
        if (state == State.BROWN || state == State.PARTYPOOPMODE) {
            poopMode = !poopMode;
        }
    }
}
