// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class RGB extends SubsystemBase {

    /** Creates a new RGB. */
    private SerialPort laPuerta;
    private StringBuilder data;


    public RGB() {
      laPuerta = new SerialPort(9600, SerialPort.Port.kMXP);
      data = new StringBuilder(/*DriverStation.getAlliance().get() == Alliance.Red ? "Red" : "Blue"*/ "testset");
    }

    public void changeString(String str) {
      data.setLength(0);
      data.append(str);
    }

    @Override
    public void periodic() {
      laPuerta.writeString(data.toString());
        // This method will be called once per scheduler run
    }
}
