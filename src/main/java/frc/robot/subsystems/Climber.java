// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxCurrent;

public class Climber extends SubsystemBase {

    public CANSparkMaxCurrent climberMotor;
    public RelativeEncoder intakEncoder;
    public SparkPIDController pidIntakeEncoder;

    /** Creates a new intake. */
    public Climber() {
        climberMotor = new CANSparkMaxCurrent(Constants.ClimberConsts.CLIMBER, MotorType.kBrushless);
        climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakEncoder = climberMotor.getEncoder();
    }

    @Override
    public void periodic() {}

    public void climberStarting(double speed) {
        climberMotor.set(speed);
    }

    public void climberStop() {
        climberMotor.set(0);
    }
}
