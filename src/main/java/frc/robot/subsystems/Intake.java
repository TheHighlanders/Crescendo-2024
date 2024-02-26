// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RGB.State;
import frc.robot.util.CANSparkMaxCurrent;

public class Intake extends SubsystemBase {

    private CANSparkMaxCurrent intakeMotor;

    private DigitalInput beamBreak;

    private boolean hasGamePiece;
    private boolean override = false;

    public Intake() {
        this.hasGamePiece = true;

        beamBreak = new DigitalInput(0);

        intakeMotor = new CANSparkMaxCurrent(Constants.Intake.INTAKE, MotorType.kBrushless);

        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intakeMotor.setSpikeCurrentLimit(
            Constants.Intake.IntakeCurrentLimit.kLimitToAmps,
            Constants.Intake.IntakeCurrentLimit.kMaxSpikeTime,
            Constants.Intake.IntakeCurrentLimit.kMaxSpikeAmps,
            Constants.Intake.IntakeCurrentLimit.kSmartLimit
        );
    }

    @Override
    public void periodic() {
        hasGamePiece = beamBreak.get();

        RobotContainer.s_RGB.setLED((hasGamePiece ? State.ORANGESOLID : State.ORANGEBLINK));

        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece);
    }

    public boolean gamePieceDetectionOverride() {
        override = !override;
        DriverStation.reportWarning("Gamepiece override " + override, false);
        return hasGamePiece();
    }

    public boolean hasGamePiece() {
        return hasGamePiece == !override;
    }

    public void intakeStop() {
        intakeMotor.set(0);
    }

    public void intakeForward() {
        intakeMotor.set(1);
    }

    public void intakeReverse() {
        intakeMotor.set(-1);
    }
}
