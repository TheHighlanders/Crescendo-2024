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

public class Intake extends SubsystemBase {

    public CANSparkMaxCurrent intakeMotor;
    public RelativeEncoder intakeEncoder;
    public SparkPIDController pidIntakeController;

    public boolean hasGamePiece;

    public Pivot pivot;

    public Intake(Pivot pivot) {
        this.hasGamePiece = true;

        this.pivot = pivot;

        intakeMotor = new CANSparkMaxCurrent(Constants.Intake.INTAKE, MotorType.kBrushless);

        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intakeEncoder = intakeMotor.getEncoder();

        intakeMotor.setSpikeCurrentLimit(
            Constants.Intake.IntakeCurrentLimit.kLimitToAmps,
            Constants.Intake.IntakeCurrentLimit.kMaxSpikeTime,
            Constants.Intake.IntakeCurrentLimit.kMaxSpikeAmps,
            Constants.Intake.IntakeCurrentLimit.kSmartLimit
        );
    }

    @Override
    public void periodic() {
        if (pivot.getIntakeDeploy()) {
            hasGamePiece = intakeMotor.getOutputCurrent() >= Constants.Intake.kGamePieceDetectionCurrent;
        }

        if (hasGamePiece) {
            intakeStop();
        }
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    public void intakeStop() {
        intakeMotor.set(0);
    }

    public void intakeStarting() {
        intakeMotor.set(1);
    }

    public void intakeStartout() {
        intakeMotor.set(-1);
    }
}
