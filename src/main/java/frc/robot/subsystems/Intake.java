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

    // public static RGB s_RGB = new RGB();

    private CANSparkMaxCurrent intakeMotor;

    private DigitalInput beamBreak;

    private boolean hasGamePiece;

    public Intake() {
        this.hasGamePiece = true;

        beamBreak = new DigitalInput(Constants.Intake.kIntakeBeamBreakDIOPin);

        intakeMotor = new CANSparkMaxCurrent(Constants.Intake.INTAKE, MotorType.kBrushed);

        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intakeMotor.setSpikeCurrentLimit(
                Constants.Intake.IntakeCurrentLimit.kLimitToAmps,
                Constants.Intake.IntakeCurrentLimit.kMaxSpikeTime,
                Constants.Intake.IntakeCurrentLimit.kMaxSpikeAmps,
                Constants.Intake.IntakeCurrentLimit.kSmartLimit);
    }

    @Override
    public void periodic() {
        // Checks if we have gained a piece, and schedules retraction
        if (hasGamePiece == beamBreak.get() && hasGamePiece == false && !DriverStation.isAutonomous()) {
            DriverStation.reportWarning("Intake Retraction AutoCommanded", true);
            RobotContainer.intakeRetract.schedule();
        }

        hasGamePiece = !beamBreak.get();

        if (hasGamePiece) {
            RobotContainer.s_RGB.setLED(State.ORANGESOLID); // 6 is solid orange meaning the robot has a note.
        } else {
            RobotContainer.s_RGB.setLED(State.ORANGEBLINK); // 5 is orange blink meaning there is no note.
        }

        // RobotContainer.s_RGB.setLED((hasGamePiece ? State.ORANGESOLID :
        // State.ORANGEBLINK));

        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece);
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    public void intakeStop() {
        DriverStation.reportWarning("Intake stop", false);
        intakeMotor.set(0);
    }

    public void intakeForward() {
        DriverStation.reportWarning("Intake forward", false);
        intakeMotor.set(1);
    }

    public void intakeReverse() {
        DriverStation.reportWarning("Intake reverse", false);
        intakeMotor.set(-1);
    }
}
