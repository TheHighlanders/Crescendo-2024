// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxCurrent;

public class intake extends SubsystemBase {

  public CANSparkMaxCurrent intakeMotor;
  public RelativeEncoder intakEncoder;
  public SparkPIDController pidIntakeEncoder;

  /** Creates a new intake. */
  private CANSparkMax intake = new CANSparkMax(Constants.intakeconstants.INTAKE, MotorType.kBrushless);

  public intake() {
    intakeMotor = new CANSparkMaxCurrent(Constants.intakeconstants.INTAKE,MotorType.kBrushless);
    intakEncoder = intakeMotor.getEncoder();
    pidIntakeEncoder = intakeMotor.getPIDController();
    pidIntakeEncoder.setOutputRange(
      Constants.intakeconstants.pidValues.minOut,
      Constants.intakeconstants.pidValues.maxOut
    );
    pidIntakeEncoder.setP(Constants.intakeconstants.pidValues.kP);
    pidIntakeEncoder.setI(Constants.intakeconstants.pidValues.kI);
    pidIntakeEncoder.setD(Constants.intakeconstants.pidValues.kD);
    pidIntakeEncoder.setIMaxAccum(
      Constants.intakeconstants.pidValues.iMaxAccum,
      Constants.intakeconstants.pidValues.slotID
    );



  }

  @Override
  public void periodic() {

  }

  public void intakestop() {
    intake.set(0);
  }

  public void intakeStarting() {
    intake.set(.8);
  }

  public void intakeStartout() {
    intake.set(-.8);
  }

}
