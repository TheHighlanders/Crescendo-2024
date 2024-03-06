// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private RelativeEncoder shooterExtensionEncoder;
    private SparkPIDController pidShooterExtensionController;

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
        shooterExtensionEncoder = shooterAngleMotor.getEncoder();
        shooterExtensionEncoder.setPositionConversionFactor(Shooter.Pivot.actuatorConst.inchesToRotationsConversion);
        pidShooterExtensionController = shooterAngleMotor.getPIDController();
        pidShooterExtensionController.setOutputRange(Shooter.Pivot.PIDValues.minOut, Shooter.Pivot.PIDValues.maxOut);
        pidShooterExtensionController.setP(Shooter.Pivot.PIDValues.kP);
        pidShooterExtensionController.setI(Shooter.Pivot.PIDValues.kI);
        pidShooterExtensionController.setD(Shooter.Pivot.PIDValues.kD);
        pidShooterExtensionController.setIMaxAccum(Shooter.Pivot.PIDValues.iMaxAccum, Shooter.Pivot.slotID);

        shooterAngleMotor.setSpikeCurrentLimit(
            Shooter.Pivot.ArmCurrentLimit.kLimitToAmps,
            Shooter.Pivot.ArmCurrentLimit.kMaxSpikeTime,
            Shooter.Pivot.ArmCurrentLimit.kMaxSpikeAmps,
            Shooter.Pivot.ArmCurrentLimit.kSmartLimit
        );

        shooterExtensionEncoder.setPosition(Shooter.Pivot.initExtension);
    }

    public InterpolatableShotData interpolate(double dist) {
        return iTreeMapContainer.interpolate(dist);
    }

    // Encoder offsets
    @Deprecated
    public double getShooterAbsolutePosition() {
        return ((absolShooter.get() * 360) - Shooter.Pivot.absoluteEncoderOffset) * Shooter.Pivot.inversionFactor;
    }

    public double getIntakeAbsolutePosition() {
        return ((absolIntake.get() * 360) - Intake.Pivot.absoluteEncoderOffset) * Intake.Pivot.inversionFactor;
    }

    public double getShooterRelativePosition() {
        return shooterExtensionEncoder.getPosition(); //convert to deg
    }

    public double getIntakeRelativePosition() {
        return intakeAngleEncoder.getPosition(); //convert to deg
    }

    /**Aligns both intake and shooter to a given angle */
    public boolean alignPivot(DoubleSupplier Extension) {
        try {
            double extSupplied = Extension.getAsDouble();
            alignShooterToExtension(extSupplied);
            // TODO: change this to use approx math
            alignIntakeToAngle(convertDistanceInchesToAngleDeg(extSupplied), false);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public void alignShooterToExtension(double Extension) {
        cachedSetpointShooter = Extension;
        pidShooterExtensionController.setReference(cachedSetpointShooter, CANSparkMax.ControlType.kPosition);
    }

    public void alignIntakeToAngle(double Angle, boolean deployed) {
        cachedSetpointIntake = Angle;
        pidIntakeAngleController.setReference(Angle, CANSparkMax.ControlType.kPosition);
        intakeDeployed = deployed;
    }

    /** Moves intake back into robot, matching its angle to the shooter */
    public void intakeIn() {
        // TODO: fix this to align to the shooter using extension
        alignIntakeToAngle(convertDistanceInchesToAngleDeg(getShooterRelativePosition()), false);
    }

    public void intakeOut() {
        alignIntakeToAngle(Intake.Pivot.intakeOutAngle, true);
    }

    // Put shooter to avg shootig angle and align the Pivot
    public void readyPositions() {
        alignIntakeToAngle(Intake.Pivot.readyAngle, false);
        alignShooterToExtension(Shooter.Pivot.readyInches);
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
            Math.abs(cachedSetpointShooter - getShooterRelativePosition()) <= Shooter.Pivot.shooterExtensionDeadzone &&
            Math.abs(shooterExtensionEncoder.getVelocity()) <= Shooter.Pivot.actuatorConst.extensionVelocityDeadzone
        );
    }

    public void driveShooterAngleManual(double speed) {
        shooterAngleMotor.set(speed);
    }

    public void driveShooterAngleManual(DoubleSupplier speed) {
        shooterAngleMotor.set(speed.getAsDouble());
    }

    public void stopShooterAngleNoHold() {
        shooterAngleMotor.set(0d);
    }

    public boolean getIntakeDeploy() {
        return intakeDeployed;
    }

    public double convertDistanceInchesToAngleDeg(double dist) {
        return (
            Math.toDegrees(
                Math.acos(
                    (
                        Math.pow(dist, 2) -
                        Math.pow(Shooter.Pivot.actuatorConst.actuatorHypot, 2) -
                        Math.pow(Shooter.Pivot.shooterBaseToArmPivotAxis, 2) +
                        Math.pow(Shooter.Pivot.actuatorConst.pivotToActuatorCenterAxis, 2)
                    ) /
                    (-2 * Shooter.Pivot.shooterBaseToArmPivotAxis * Shooter.Pivot.actuatorConst.actuatorHypot)
                ) -
                Shooter.Pivot.actuatorConst.actuatorAngleBaseDist
            ) + Shooter.Pivot.actuatorConst.secretAngleDeg
        );
    }

    @Override
    public void periodic() {
        shooterAngleMotor.periodicLimit();
        intakeAngleMotor.periodicLimit();

        SmartDashboard.putNumber("Intake Angle Value absolute", getIntakeAbsolutePosition());
        SmartDashboard.putNumber("Intake Angle Value absolute no offset", (absolIntake.get() * 360));

        SmartDashboard.putNumber("Shooter extension", shooterExtensionEncoder.getPosition());
        SmartDashboard.putBoolean("Shooter at setpoint", shooterAtSetpoint());

        SmartDashboard.putNumber("Shooter Inferred Angle", convertDistanceInchesToAngleDeg(shooterExtensionEncoder.getPosition()));

        SmartDashboard.putNumber("Shooter Measured Angle Value", getShooterRelativePosition());
    }
}
