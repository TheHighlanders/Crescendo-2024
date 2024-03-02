// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
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

    private DutyCycleEncoder absolShooter;
    private DutyCycleEncoder absolIntake;

    private double cachedSetpointIntake = 0;
    private double cachedSetpointShooter = 0;

    private boolean intakeDeployed;

    public Pivot() {
        iTreeMapContainer = new InterpolatingShotTreeMapContainer();
        /*----------------------------------------------------------------------------*/
        /* Intake */
        /*----------------------------------------------------------------------------*/

        absolIntake = new DutyCycleEncoder(Intake.Pivot.kAbsolDutyCycleDIOPin);

        intakeAngleMotor = new CANSparkMaxCurrent(Intake.Pivot.INTAKE, MotorType.kBrushless);
        intakeAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeAngleEncoder = intakeAngleMotor.getEncoder();
        intakeAngleEncoder.setPositionConversionFactor(Intake.Pivot.intakePivotRatio);
        pidIntakeAngleController = intakeAngleMotor.getPIDController();
        pidIntakeAngleController.setOutputRange(Intake.Pivot.PIDValues.minOut, Intake.Pivot.PIDValues.maxOut);
        pidIntakeAngleController.setP(Intake.Pivot.PIDValues.kP);
        pidIntakeAngleController.setI(Intake.Pivot.PIDValues.kI);
        pidIntakeAngleController.setD(Intake.Pivot.PIDValues.kD);
        pidIntakeAngleController.setIMaxAccum(Intake.Pivot.PIDValues.iMaxAccum, Intake.Pivot.slotID);

        intakeAngleMotor.setSpikeCurrentLimit(
            Intake.Pivot.ArmCurrentLimit.kLimitToAmps,
            Intake.Pivot.ArmCurrentLimit.kMaxSpikeTime,
            Intake.Pivot.ArmCurrentLimit.kMaxSpikeAmps,
            Intake.Pivot.ArmCurrentLimit.kSmartLimit
        );

        intakeAngleEncoder.setPosition(getIntakeAbsolutePosition());

        intakeDeployed = false;

        /*----------------------------------------------------------------------------*/
        /* Shooter */
        /*----------------------------------------------------------------------------*/
        absolShooter = new DutyCycleEncoder(Shooter.Pivot.kAbsolDutyCycleDIOPin);

        shooterAngleMotor = new CANSparkMaxCurrent(Shooter.Pivot.SHOOTER, MotorType.kBrushless);
        shooterAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterAngleEncoder = shooterAngleMotor.getEncoder();
        shooterAngleEncoder.setPositionConversionFactor(Shooter.Pivot.actuatorConst.inchesToRotationsConversion);
        pidShooterAngleController = shooterAngleMotor.getPIDController();
        shooterAngleEncoder.setPositionConversionFactor(Shooter.Pivot.shooterPivotRatio);
        pidShooterAngleController.setOutputRange(Shooter.Pivot.PIDValues.minOut, Shooter.Pivot.PIDValues.maxOut);
        pidShooterAngleController.setP(Shooter.Pivot.PIDValues.kP);
        pidShooterAngleController.setI(Shooter.Pivot.PIDValues.kI);
        pidShooterAngleController.setD(Shooter.Pivot.PIDValues.kD);
        pidShooterAngleController.setIMaxAccum(Shooter.Pivot.PIDValues.iMaxAccum, Shooter.Pivot.slotID);

        shooterAngleMotor.setSpikeCurrentLimit(
            Shooter.Pivot.ArmCurrentLimit.kLimitToAmps,
            Shooter.Pivot.ArmCurrentLimit.kMaxSpikeTime,
            Shooter.Pivot.ArmCurrentLimit.kMaxSpikeAmps,
            Shooter.Pivot.ArmCurrentLimit.kSmartLimit
        );

        shooterAngleEncoder.setPosition(Constants.Shooter.Pivot.initAngle);
    }

    public InterpolatableShotData interpolate(double dist) {
        return iTreeMapContainer.interpolate(dist);
    }

    // Encoder offsets
    public double getShooterAbsolutePosition() {
        return ((absolShooter.get() * 360) - Shooter.Pivot.absoluteEncoderOffset) * Shooter.Pivot.inversionFactor;
    }

    public double getIntakeAbsolutePosition() {
        return ((absolIntake.get() * 360) - Intake.Pivot.absoluteEncoderOffset) * Intake.Pivot.inversionFactor;
    }

    public double getShooterRelativePosition() {
        return convertDistanceInchesToAngleRad(shooterAngleEncoder.getPosition()) * 180 / Math.PI; //convert to deg
    }

    public double getIntakeRelativePosition() {
        return convertDistanceInchesToAngleRad(intakeAngleEncoder.getPosition()) * 180 / Math.PI; //convert to deg
    }

    /**Aligns both intake and shooter to a given angle */
    public boolean alignPivot(DoubleSupplier angle) {
        try {
            double angleSupplied = angle.getAsDouble();
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public void alignShooterToAngle(double angle) {
        cachedSetpointShooter = convertAngleToDistanceInches(angle);
        pidShooterAngleController.setReference(cachedSetpointShooter, CANSparkMax.ControlType.kPosition);
    }

    public void alignIntakeToAngle(double angle, boolean deployed) {
        cachedSetpointIntake = angle;
        pidIntakeAngleController.setReference(angle, CANSparkMax.ControlType.kPosition);
        intakeDeployed = deployed;
    }

    /** Moves intake back into robot, matching its angle to the shooter */
    public void intakeIn() {
        alignIntakeToAngle(getShooterRelativePosition(), false);
    }

    public void intakeOut() {
        alignIntakeToAngle(Intake.Pivot.intakeOutAngle, true);
    }

    // Put shooter to avg shootig angle and align the Pivot
    public void readyPositions() {
        alignIntakeToAngle(Intake.Pivot.readyAngle, false);
        alignShooterToAngle(Shooter.Pivot.readyAngle);

    }

    public boolean atSetpoints() {
        return intakeAtSetpoint() && shooterAtSetpoint();
    }

    // must be still at specified position to be true
    public boolean intakeAtSetpoint() {
        return (
            Math.abs(cachedSetpointIntake - getIntakeAbsolutePosition()) <= Intake.Pivot.intakeAngleDeadzone &&
            Math.abs(intakeAngleEncoder.getVelocity()) == 0
        );
    }

    // must be still at specified position to be true
    public boolean shooterAtSetpoint() {
        return (
            Math.abs(cachedSetpointShooter - getShooterRelativePosition()) <= Shooter.Pivot.shooterAngleDeadzone &&
            Math.abs(shooterAngleEncoder.getVelocity()) == 0
        );
    }

    public void driveShooterAngleManual(double speed) {
        shooterAngleMotor.set(speed);
    }

    public void driveShooterAngleManual(DoubleSupplier speed) {
        shooterAngleMotor.set(speed.getAsDouble());
    }

    public void stopShooterAngle() {
        shooterAngleMotor.set(0d);
    }

    // used for linear actuator conversion
    public double convertAngleToDistanceInches(double angle) {
        angle = Math.toRadians(angle);
        return Math.sqrt(
            Math.pow(Shooter.Pivot.actuatorConst.actuatorHypot, 2) +
            Math.pow(Shooter.Pivot.shooterBaseToArmPivotAxis, 2) -
            Math.pow(Shooter.Pivot.actuatorConst.pivotToActuatorCenterAxis, 2) -
            (
                2 *
                Shooter.Pivot.shooterBaseToArmPivotAxis *
                Shooter.Pivot.actuatorConst.actuatorHypot *
                Math.cos(angle + Shooter.Pivot.actuatorConst.actuatorAngleBaseDist)
            )
        );
    }

    public double convertDistanceInchesToAngleRad(double dist) {
        return (
            Math.acos(
                (
                    Math.pow(dist, 2) -
                    Math.pow(Shooter.Pivot.actuatorConst.actuatorHypot, 2) -
                    Math.pow(Shooter.Pivot.shooterBaseToArmPivotAxis, 2) +
                    Math.pow(Shooter.Pivot.actuatorConst.pivotToActuatorCenterAxis, 2)
                ) /
                (-2 * Shooter.Pivot.actuatorConst.actuatorHypot * dist)
            ) -
            Shooter.Pivot.actuatorConst.actuatorAngleBaseDist
        );
    }

    public boolean getIntakeDeploy() {
        return intakeDeployed;
    }

    @Override
    public void periodic() {
        shooterAngleMotor.periodicLimit();
        intakeAngleMotor.periodicLimit();

        SmartDashboard.putNumber("Intake Angle Value", getIntakeAbsolutePosition());
        //SmartDashboard.putNumber("Shooter Composed Angle Value", getShooterRelativePosition() + getIntakeAbsolutePosition());
        SmartDashboard.putNumber("Shooter Measured Angle Value", getShooterRelativePosition());
        SmartDashboard.putNumber("Intake Rots", absolIntake.get());
        SmartDashboard.putNumber("Shooter Rots", absolShooter.get());
        SmartDashboard.putNumber("Sum of Rots", absolIntake.get() - absolShooter.get());
        SmartDashboard.putNumber("Relative Shooter Encoder Pivot", shooterAngleEncoder.getPosition());
    }
}
