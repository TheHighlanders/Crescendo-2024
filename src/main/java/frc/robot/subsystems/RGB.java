// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGB extends SubsystemBase {

    /** Creates a new RGB. */
    private SerialPort elPuerto;

    public RGB() {
        elPuerto = new SerialPort(9600, SerialPort.Port.kMXP);
        elPuerto.writeString("5\n");
    }

    public void changeString(String str) {
        elPuerto.writeString(str + "\n");
    }

    @Override
    public void periodic() {
        // DriverStation.reportWarning("something", false);
        // This method will be called once per scheduler run
    }
}
