// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

    private PIDController differentialPidController;
    private CANSparkMaxCurrent intakeAngleMotor;
    private RelativeEncoder intakeAngleEncoder;
    private SparkPIDController pidIntakeAngleController;

    private CANSparkMaxCurrent shooterAngleMotor;
    private RelativeEncoder shooterExtensionEncoder;
    private SparkPIDController pidShooterExtensionController;

    private DutyCycleEncoder absolShooter;
    private DutyCycleEncoder absolIntake;

    private double cachedSetpointShooter = 0;

    private boolean intakeDeployed;

    public static Command intakeShooterCommand;

    public Pivot() {
        iTreeMapContainer = new InterpolatingShotTreeMapContainer();
        /*----------------------------------------------------------------------------*/
        /* Intake */
        /*----------------------------------------------------------------------------*/

        absolIntake = new DutyCycleEncoder(Intake.Pivot.kAbsolDutyCycleDIOPin);

        intakeAngleMotor = new CANSparkMaxCurrent(Intake.Pivot.INTAKE, MotorType.kBrushless);
        intakeAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeAngleEncoder = intakeAngleMotor.getEncoder();
        intakeAngleEncoder.setPositionConversionFactor(Intake.Pivot.intakePivotRatio);
        pidIntakeAngleController = intakeAngleMotor.getPIDController();
        pidIntakeAngleController.setOutputRange(Intake.Pivot.PIDValues.minOut, Intake.Pivot.PIDValues.maxOut);
        pidIntakeAngleController.setP(Intake.Pivot.PIDValues.kP);
        pidIntakeAngleController.setI(Intake.Pivot.PIDValues.kI);
        pidIntakeAngleController.setD(Intake.Pivot.PIDValues.kD);
        pidIntakeAngleController.setIMaxAccum(Intake.Pivot.PIDValues.iMaxAccum, Intake.Pivot.slotID);

        differentialPidController =
            new PIDController(
                Intake.Pivot.PIDValues.deviationPID.kP,
                Intake.Pivot.PIDValues.deviationPID.kI,
                Intake.Pivot.PIDValues.deviationPID.kD,
                0.02
            );
        differentialPidController.setTolerance(Intake.Pivot.PIDValues.deviationPID.posTolerance, Intake.Pivot.PIDValues.deviationPID.velTolerance);

        intakeAngleMotor.setSpikeCurrentLimit(
            Intake.Pivot.ArmCurrentLimit.kLimitToAmps,
            Intake.Pivot.ArmCurrentLimit.kMaxSpikeTime,
            Intake.Pivot.ArmCurrentLimit.kMaxSpikeAmps,
            Intake.Pivot.ArmCurrentLimit.kSmartLimit
        );

        intakeAngleEncoder.setPosition(Intake.Pivot.intakeInit);

        intakeShooterCommand =
            new FunctionalCommand(
                () -> {},
                () ->
                    intakeAngleMotor.set(
                        -MathUtil.clamp(
                            differentialPidController.calculate(getPositionDiffrential(), 0),
                            Intake.Pivot.PIDValues.minOut,
                            Intake.Pivot.PIDValues.maxOut
                        )
                    ), //pidIntakeAngleController.setReference(-(differentialPidController.calculate(getPositionDiffrential(), 0)),ControlType.kDutyCycle);
                v -> intakeAngleHold(),
                () -> intakeAtSetpointShooter()
            );

        intakeDeployed = false;

        /*----------------------------------------------------------------------------*/
        /* Shooter */
        /*----------------------------------------------------------------------------*/
        absolShooter = new DutyCycleEncoder(Shooter.Pivot.kAbsolDutyCycleDIOPin);

        shooterAngleMotor = new CANSparkMaxCurrent(Shooter.Pivot.SHOOTER, MotorType.kBrushless);
        shooterAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
        InterpolatableShotData i = iTreeMapContainer.interpolate(dist);
        if (i == null) {
            return iTreeMapContainer.interpolate(1.5);
        }
        return i;
    }

    public InterpolatableShotData interpolate(DoubleSupplier dist) {
        return interpolate(dist.getAsDouble());
    }

    // Encoder offsets
    public double getPositionDiffrential() {
        return ((absolShooter.get() * 360) * Shooter.Pivot.inversionFactor) - Shooter.Pivot.absoluteEncoderOffset;
    }

    public void goLimp() {
        pidShooterExtensionController.setReference(0, ControlType.kDutyCycle);
    }

    public double getIntakeAbsolutePosition() {
        return ((absolIntake.get() * 360) - Intake.Pivot.absoluteEncoderOffset) * Intake.Pivot.inversionFactor;
    }

    public double getShooterRelativePosition() {
        return shooterExtensionEncoder.getPosition();
    }

    public double getIntakeRelativePosition() {
        return intakeAngleEncoder.getPosition();
    }

    public Command retractIntake() {
        return new FunctionalCommand(
            () -> {},
            () ->
                intakeAngleMotor.set(
                    MathUtil.clamp(
                        differentialPidController.calculate(getPositionDiffrential(), 0),
                        Intake.Pivot.PIDValues.minOut,
                        Intake.Pivot.PIDValues.maxOut
                    )
                ),
            v -> intakeAngleHold(),
            () -> intakeAtSetpointShooter()
        );
    }

    /**Aligns both intake and shooter to a given angle */
    public boolean alignPivot(DoubleSupplier Extension) {
        if (Constants.diagnosticMode) {
            DriverStation.reportWarning("In Align Pivot Pivto Subsystem", true);
        }
        try {
            double extSupplied = Extension.getAsDouble();
            if (Constants.diagnosticMode) {
                DriverStation.reportWarning("After Double", false);
            }
            alignShooterToExtension(extSupplied);
            alignIntakeToShooter();
            return true;
        } catch (Exception e) {
            DriverStation.reportWarning("Arm Threw an Exception", true);
            return false;
        }
    }

    public boolean alignPivot(double extension) {
        return alignPivot(() -> extension);
    }

    public void alignShooterToExtension(double Extension) {
        cachedSetpointShooter = Extension;
        pidShooterExtensionController.setReference(cachedSetpointShooter, CANSparkMax.ControlType.kPosition);

        if (Constants.diagnosticMode) {
            SmartDashboard.putNumber("Shooter setpoint", cachedSetpointShooter);
        }
    }

    public void alignIntakeToShooter() {
        if (DriverStation.isTeleop()) {
            intakeShooterCommand.schedule();
        }

        if (Constants.diagnosticMode) {
            DriverStation.reportWarning("Set Intake Setpoint Shooter", false);
        }
    }

    public void alignIntakeToGround() {
        intakeShooterCommand.cancel();
        if (Constants.diagnosticMode) {
            DriverStation.reportWarning("Set Intake Setpoint Ground", false);
        }
        pidIntakeAngleController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    public void alignIntakeToAmp() {
        intakeShooterCommand.cancel();
        if (Constants.diagnosticMode) {
            DriverStation.reportWarning("Set Intake Setpoint Amp", false);
        }
        pidIntakeAngleController.setReference(frc.robot.Constants.Intake.Pivot.intakeAtAmp, CANSparkMax.ControlType.kPosition);
    }

    // Put shooter to avg shootig angle and align the Pivot
    public void readyPositions() {
        alignIntakeToShooter();
        alignShooterToExtension(Shooter.Pivot.readyInches);
    }

    public boolean atSetpointsAtShooter() {
        return intakeAtSetpointShooter() && shooterAtSetpoint();
    }

    // must be still at specified position to be true
    public boolean intakeAtSetpointGround() {
        return (Math.abs(getIntakeRelativePosition()) <= Intake.Pivot.intakeAngleDeadzone);
    }

    public boolean intakeAtSetpointShooter() {
        return differentialPidController.atSetpoint();
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

    public void driveIntakeManual(double speed) {
        intakeAngleMotor.set(speed);
    }

    public void stopShooterAngleNoHold() {
        shooterAngleMotor.set(0d);
    }

    public void shooterAngleHold() {
        pidShooterExtensionController.setReference(shooterExtensionEncoder.getPosition(), ControlType.kPosition);
    }

    public void intakeAngleHold() {
        pidIntakeAngleController.setReference(intakeAngleEncoder.getPosition(), ControlType.kPosition);
    }

    public boolean getIntakeDeploy() {
        return intakeDeployed;
    }

    public void setShooterCoastMode() {
        shooterAngleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setShooterBrakeMode() {
        shooterAngleMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        shooterAngleMotor.periodicLimit();
        intakeAngleMotor.periodicLimit();
        if (Constants.diagnosticMode) {
            SmartDashboard.putBoolean("Arm setpoint intaks", intakeAtSetpointShooter());
            SmartDashboard.putBoolean("Extension at setpoint", shooterAtSetpoint());
            SmartDashboard.putNumber("extension", shooterExtensionEncoder.getPosition());
            SmartDashboard.putNumber("AbsolIntakePos", getIntakeAbsolutePosition());
        }
    }
}
