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

public class intake extends SubsystemBase {

  public CANSparkMaxCurrent intakeMotor;
  public RelativeEncoder intakeEncoder;
  public SparkPIDController pidIntakeController;

  public intake() {
    intakeMotor =
      new CANSparkMaxCurrent(Constants.Intake.INTAKE, MotorType.kBrushless);
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeEncoder = intakeMotor.getEncoder();
    pidIntakeController = intakeMotor.getPIDController();
    pidIntakeController.setOutputRange(
      Constants.Intake.pidValues.minOut,
      Constants.Intake.pidValues.maxOut
    );
    pidIntakeController.setP(Constants.Intake.pidValues.kP);
    pidIntakeController.setI(Constants.Intake.pidValues.kI);
    pidIntakeController.setD(Constants.Intake.pidValues.kD);
    pidIntakeController.setIMaxAccum(
      Constants.Intake.pidValues.iMaxAccum,
      Constants.Intake.slotID
    );
    pidIntakeController.setSmartMotionMaxVelocity(
      Constants.Intake.maxVel,
      Constants.Intake.slotID
    );
    pidIntakeController.setSmartMotionMinOutputVelocity(
      Constants.Intake.minVel,
      Constants.Intake.slotID
    );
    pidIntakeController.setSmartMotionMaxAccel(
      Constants.Intake.maxAcc,
      Constants.Intake.slotID
    );
    pidIntakeController.setSmartMotionAllowedClosedLoopError(
      Constants.Intake.allowedErr,
      Constants.Intake.slotID
    );

    intakeMotor.setSpikeCurrentLimit(
      Constants.Intake.IntakeCurrentLimit.kLimitToAmps,
      Constants.Intake.IntakeCurrentLimit.kMaxSpikeTime,
      Constants.Intake.IntakeCurrentLimit.kMaxSpikeAmps,
      Constants.Intake.IntakeCurrentLimit.kSmartLimit
    );
  }

  @Override
  public void periodic() {}

  public void intakestop() {
    pidIntakeController.setReference(0, CANSparkMax.ControlType.kSmartVelocity);
  }

  public void intakeStarting() {
    pidIntakeController.setReference(1, CANSparkMax.ControlType.kSmartVelocity);
  }

  public void intakeStartout() {
    pidIntakeController.setReference(
      -1,
      CANSparkMax.ControlType.kSmartVelocity
    );
  }
}
