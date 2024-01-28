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
import frc.robot.util.InterpolatableShotData;
import frc.robot.util.InterpolatingShotTreeMapContainer;
import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {

    private InterpolatingShotTreeMapContainer iTreeMapContainer;

    private CANSparkMaxCurrent intakeAngleMotor;
    private RelativeEncoder intakeAngleEncoder;
    private SparkPIDController pidIntakeAngleController;

    private CANSparkMaxCurrent shooterAngleMotor;
    private RelativeEncoder shooterAngleEncoder;
    private SparkPIDController pidShooterAngleController;

    private double cachedSetpointIntake = 0;
    private double cachedSetpointShooter = 0;

    public Pivot() {
        iTreeMapContainer = new InterpolatingShotTreeMapContainer();
        /*----------------------------------------------------------------------------*/
        /* Intake */
        /*----------------------------------------------------------------------------*/
        intakeAngleMotor = new CANSparkMaxCurrent(Constants.Pivot.INTAKE, MotorType.kBrushless);
        intakeAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeAngleEncoder = intakeAngleMotor.getEncoder();
        intakeAngleEncoder.setPositionConversionFactor(Constants.Pivot.intakePivotRatio);
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
        /* Shooter */
        /*----------------------------------------------------------------------------*/

        shooterAngleMotor = new CANSparkMaxCurrent(Constants.Pivot.INTAKE, MotorType.kBrushless);
        shooterAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterAngleEncoder = shooterAngleMotor.getEncoder();
        shooterAngleEncoder.setPositionConversionFactor(
            Constants.Pivot.actuatorConst.inchesToRotationsConversion
        );
        pidShooterAngleController = shooterAngleMotor.getPIDController();
        shooterAngleEncoder.setPositionConversionFactor(Constants.Pivot.shooterPivotRatio);
        pidShooterAngleController.setOutputRange(
            Constants.Pivot.pidValuesShooter.minOut,
            Constants.Pivot.pidValuesShooter.maxOut
        );
        pidShooterAngleController.setP(Constants.Pivot.pidValuesShooter.kP);
        pidShooterAngleController.setI(Constants.Pivot.pidValuesShooter.kI);
        pidShooterAngleController.setD(Constants.Pivot.pidValuesShooter.kD);
        pidShooterAngleController.setIMaxAccum(
            Constants.Pivot.pidValuesShooter.iMaxAccum,
            Constants.Pivot.slotID
        );
        pidShooterAngleController.setSmartMotionMaxVelocity(
            Constants.Pivot.SmartMotionCoefficientsShooter.maxVel,
            Constants.Pivot.slotID
        );
        pidShooterAngleController.setSmartMotionMinOutputVelocity(
            Constants.Pivot.SmartMotionCoefficientsShooter.minVel,
            Constants.Pivot.slotID
        );
        pidShooterAngleController.setSmartMotionMaxAccel(
            Constants.Pivot.SmartMotionCoefficientsShooter.maxAcc,
            Constants.Pivot.slotID
        );
        pidShooterAngleController.setSmartMotionAllowedClosedLoopError(
            Constants.Pivot.SmartMotionCoefficientsShooter.allowedErr,
            Constants.Pivot.slotID
        );

        shooterAngleMotor.setSpikeCurrentLimit(
            Constants.Pivot.ShooterArmCurrentLimit.kLimitToAmps,
            Constants.Pivot.ShooterArmCurrentLimit.kMaxSpikeTime,
            Constants.Pivot.ShooterArmCurrentLimit.kMaxSpikeAmps,
            Constants.Pivot.ShooterArmCurrentLimit.kSmartLimit
        );
    }

    public InterpolatableShotData interpolate(double dist) {
        return iTreeMapContainer.interpolate(dist);
    }

    public boolean alignPivot(DoubleSupplier angle) {
        try {
            double angleSupplied = angle.getAsDouble();
            cachedSetpointShooter = convertAngleToDistanceInches(angleSupplied);
            cachedSetpointIntake = angleSupplied;
            pidIntakeAngleController.setReference(
                angle.getAsDouble(),
                CANSparkMax.ControlType.kPosition
            );
            pidShooterAngleController.setReference(
                angle.getAsDouble(),
                CANSparkMax.ControlType.kPosition
            );
            // call swerve subsystem alignAngle pass in angle

            return true;
        } catch (Exception e) {
            // TODO: pass e to error logger
            return false;
        }
    }

    public void intakeOut() {
        pidIntakeAngleController.setReference(
            Constants.Pivot.intakeOutAngle,
            CANSparkMax.ControlType.kPosition
        );
    }

    // Put shooter to avg shootig angle and align the Pivot
    public void readyPositions() {
        pidIntakeAngleController.setReference(
            Constants.Pivot.readyAngle,
            CANSparkMax.ControlType.kPosition
        );
        pidShooterAngleController.setReference(
            Constants.Pivot.readyAngle,
            CANSparkMax.ControlType.kPosition
        );
    }

    public boolean atSetpoints() {
        return intakeAtSetpoint() && shooterAtSetpoint();
    }

    public boolean intakeAtSetpoint() {
        return (
            Math.abs(cachedSetpointIntake - intakeAngleEncoder.getPosition()) <=
            Constants.Pivot.intakeAngleDeadzone &&
            Math.abs(intakeAngleEncoder.getVelocity()) == 0
        );
    }

    public boolean shooterAtSetpoint() {
        return (
            Math.abs(cachedSetpointShooter - shooterAngleEncoder.getPosition()) <=
            Constants.Pivot.shooterAngleDeadzone &&
            Math.abs(shooterAngleEncoder.getVelocity()) == 0
        );
    }

    public double convertAngleToDistanceInches(double angle) {
        return Math.hypot(
            Constants.Pivot.actuatorConst.actuatorBaseDistY - Math.sin(angle),
            Constants.Pivot.actuatorConst.actuatorBaseDistX - Math.cos(angle)
        );
    }

    @Override
    public void periodic() {}
}
