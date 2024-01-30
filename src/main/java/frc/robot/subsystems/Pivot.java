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
        intakeAngleMotor = new CANSparkMaxCurrent(Constants.PivotConstants.INTAKE, MotorType.kBrushless);
        intakeAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeAngleEncoder = intakeAngleMotor.getEncoder();
        intakeAngleEncoder.setPositionConversionFactor(Constants.PivotConstants.intakePivotRatio);
        pidIntakeAngleController = intakeAngleMotor.getPIDController();
        pidIntakeAngleController.setOutputRange(
            Constants.PivotConstants.pidValuesIntake.minOut,
            Constants.PivotConstants.pidValuesIntake.maxOut
        );
        pidIntakeAngleController.setP(Constants.PivotConstants.pidValuesIntake.kP);
        pidIntakeAngleController.setI(Constants.PivotConstants.pidValuesIntake.kI);
        pidIntakeAngleController.setD(Constants.PivotConstants.pidValuesIntake.kD);
        pidIntakeAngleController.setIMaxAccum(
            Constants.PivotConstants.pidValuesIntake.iMaxAccum,
            Constants.PivotConstants.slotID
        );
        pidIntakeAngleController.setSmartMotionMaxVelocity(
            Constants.PivotConstants.SmartMotionCoefficientsIntake.maxVel,
            Constants.PivotConstants.slotID
        );
        pidIntakeAngleController.setSmartMotionMinOutputVelocity(
            Constants.PivotConstants.SmartMotionCoefficientsIntake.minVel,
            Constants.PivotConstants.slotID
        );
        pidIntakeAngleController.setSmartMotionMaxAccel(
            Constants.PivotConstants.SmartMotionCoefficientsIntake.maxAcc,
            Constants.PivotConstants.slotID
        );
        pidIntakeAngleController.setSmartMotionAllowedClosedLoopError(
            Constants.PivotConstants.SmartMotionCoefficientsIntake.allowedErr,
            Constants.PivotConstants.slotID
        );

        intakeAngleMotor.setSpikeCurrentLimit(
            Constants.PivotConstants.IntakeArmCurrentLimit.kLimitToAmps,
            Constants.PivotConstants.IntakeArmCurrentLimit.kMaxSpikeTime,
            Constants.PivotConstants.IntakeArmCurrentLimit.kMaxSpikeAmps,
            Constants.PivotConstants.IntakeArmCurrentLimit.kSmartLimit
        );

        /*----------------------------------------------------------------------------*/
        /* Shooter */
        /*----------------------------------------------------------------------------*/

        shooterAngleMotor = new CANSparkMaxCurrent(Constants.PivotConstants.INTAKE, MotorType.kBrushless);
        shooterAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterAngleEncoder = shooterAngleMotor.getEncoder();
        shooterAngleEncoder.setPositionConversionFactor(
            Constants.PivotConstants.actuatorConst.inchesToRotationsConversion
        );
        pidShooterAngleController = shooterAngleMotor.getPIDController();
        shooterAngleEncoder.setPositionConversionFactor(Constants.PivotConstants.shooterPivotRatio);
        pidShooterAngleController.setOutputRange(
            Constants.PivotConstants.pidValuesShooter.minOut,
            Constants.PivotConstants.pidValuesShooter.maxOut
        );
        pidShooterAngleController.setP(Constants.PivotConstants.pidValuesShooter.kP);
        pidShooterAngleController.setI(Constants.PivotConstants.pidValuesShooter.kI);
        pidShooterAngleController.setD(Constants.PivotConstants.pidValuesShooter.kD);
        pidShooterAngleController.setIMaxAccum(
            Constants.PivotConstants.pidValuesShooter.iMaxAccum,
            Constants.PivotConstants.slotID
        );
        pidShooterAngleController.setSmartMotionMaxVelocity(
            Constants.PivotConstants.SmartMotionCoefficientsShooter.maxVel,
            Constants.PivotConstants.slotID
        );
        pidShooterAngleController.setSmartMotionMinOutputVelocity(
            Constants.PivotConstants.SmartMotionCoefficientsShooter.minVel,
            Constants.PivotConstants.slotID
        );
        pidShooterAngleController.setSmartMotionMaxAccel(
            Constants.PivotConstants.SmartMotionCoefficientsShooter.maxAcc,
            Constants.PivotConstants.slotID
        );
        pidShooterAngleController.setSmartMotionAllowedClosedLoopError(
            Constants.PivotConstants.SmartMotionCoefficientsShooter.allowedErr,
            Constants.PivotConstants.slotID
        );

        shooterAngleMotor.setSpikeCurrentLimit(
            Constants.PivotConstants.ShooterArmCurrentLimit.kLimitToAmps,
            Constants.PivotConstants.ShooterArmCurrentLimit.kMaxSpikeTime,
            Constants.PivotConstants.ShooterArmCurrentLimit.kMaxSpikeAmps,
            Constants.PivotConstants.ShooterArmCurrentLimit.kSmartLimit
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
            Constants.PivotConstants.intakeOutAngle,
            CANSparkMax.ControlType.kPosition
        );
    }

    // Put shooter to avg shootig angle and align the Pivot
    public void readyPositions() {
        pidIntakeAngleController.setReference(
            Constants.PivotConstants.readyAngle,
            CANSparkMax.ControlType.kPosition
        );
        pidShooterAngleController.setReference(
            Constants.PivotConstants.readyAngle,
            CANSparkMax.ControlType.kPosition
        );
    }

    public boolean atSetpoints() {
        return intakeAtSetpoint() && shooterAtSetpoint();
    }

    public boolean intakeAtSetpoint() {
        return (
            Math.abs(cachedSetpointIntake - intakeAngleEncoder.getPosition()) <=
            Constants.PivotConstants.intakeAngleDeadzone &&
            Math.abs(intakeAngleEncoder.getVelocity()) == 0
        );
    }

    public boolean shooterAtSetpoint() {
        return (
            Math.abs(cachedSetpointShooter - shooterAngleEncoder.getPosition()) <=
            Constants.PivotConstants.shooterAngleDeadzone &&
            Math.abs(shooterAngleEncoder.getVelocity()) == 0
        );
    }

    public double convertAngleToDistanceInches(double angle) {
        return Math.hypot(
            Constants.PivotConstants.actuatorConst.actuatorBaseDistY - Math.sin(angle),
            Constants.PivotConstants.actuatorConst.actuatorBaseDistX - Math.cos(angle)
        );
    }

    @Override
    public void periodic() {}
}
