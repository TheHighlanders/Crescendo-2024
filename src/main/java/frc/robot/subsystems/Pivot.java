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

public class Pivot extends SubsystemBase {

  public CANSparkMaxCurrent intakeAngleMotor;
  public RelativeEncoder intakeAngleEncoder;
  public SparkPIDController pidIntakeAngleController;

  public Pivot() {
    /*----------------------------------------------------------------------------*/
    /* Intake                                                                     */
    /*----------------------------------------------------------------------------*/
    intakeAngleMotor =
      new CANSparkMaxCurrent(Constants.Pivot.INTAKE, MotorType.kBrushless);
    intakeAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeAngleEncoder = intakeAngleMotor.getEncoder();
    pidIntakeAngleController = intakeAngleMotor.getPIDController();
    pidIntakeAngleController.setOutputRange(
      Constants.Pivot.pidValuesIntake.minOut,
      Constants.Pivot.pidValuesIntake.maxOut
    );
    pidIntakeAngleController.setP(Constants.Pivot.pidValuesIntake.kP);
    pidIntakeAngleController.setI(Constants.Pivot.pidValuesIntake.kI);
    pidIntakeAngleController.setD(Constants.Pivot.pidValuesIntake.kD);
    pidIntakeAngleController.setIMaxAccum(
      Constants.Pivot.pidValuesIntake.iMaxAccum,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionMaxVelocity(
      Constants.Pivot.SmartMotionCoefficientsIntake.maxVel,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionMinOutputVelocity(
      Constants.Pivot.SmartMotionCoefficientsIntake.minVel,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionMaxAccel(
      Constants.Pivot.SmartMotionCoefficientsIntake.maxAcc,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionAllowedClosedLoopError(
      Constants.Pivot.SmartMotionCoefficientsIntake.allowedErr,
      Constants.Pivot.slotID
    );

    intakeAngleMotor.setSpikeCurrentLimit(
      Constants.Pivot.IntakeArmCurrentLimit.kLimitToAmps,
      Constants.Pivot.IntakeArmCurrentLimit.kMaxSpikeTime,
      Constants.Pivot.IntakeArmCurrentLimit.kMaxSpikeAmps,
      Constants.Pivot.IntakeArmCurrentLimit.kSmartLimit
    );

    /*----------------------------------------------------------------------------*/
    /* Shooter                                                                    */
    /*----------------------------------------------------------------------------*/

    intakeAngleMotor =
      new CANSparkMaxCurrent(Constants.Pivot.INTAKE, MotorType.kBrushless);
    intakeAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeAngleEncoder = intakeAngleMotor.getEncoder();
    pidIntakeAngleController = intakeAngleMotor.getPIDController();
    pidIntakeAngleController.setOutputRange(
      Constants.Pivot.pidValuesIntake.minOut,
      Constants.Pivot.pidValuesIntake.maxOut
    );
    pidIntakeAngleController.setP(Constants.Pivot.pidValuesIntake.kP);
    pidIntakeAngleController.setI(Constants.Pivot.pidValuesIntake.kI);
    pidIntakeAngleController.setD(Constants.Pivot.pidValuesIntake.kD);
    pidIntakeAngleController.setIMaxAccum(
      Constants.Pivot.pidValuesIntake.iMaxAccum,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionMaxVelocity(
      Constants.Pivot.SmartMotionCoefficientsIntake.maxVel,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionMinOutputVelocity(
      Constants.Pivot.SmartMotionCoefficientsIntake.minVel,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionMaxAccel(
      Constants.Pivot.SmartMotionCoefficientsIntake.maxAcc,
      Constants.Pivot.slotID
    );
    pidIntakeAngleController.setSmartMotionAllowedClosedLoopError(
      Constants.Pivot.SmartMotionCoefficientsIntake.allowedErr,
      Constants.Pivot.slotID
    );

    intakeAngleMotor.setSpikeCurrentLimit(
      Constants.Pivot.IntakeArmCurrentLimit.kLimitToAmps,
      Constants.Pivot.IntakeArmCurrentLimit.kMaxSpikeTime,
      Constants.Pivot.IntakeArmCurrentLimit.kMaxSpikeAmps,
      Constants.Pivot.IntakeArmCurrentLimit.kSmartLimit
    );
  }

  public boolean alignPivot() {
    return true;
  }

  public void intakeOut() {
    
  }

  // Put shooter to avg shootig angle and align the Pivot
  public void readyPositions() {
    
  }

  @Override
  public void periodic() {}
}
