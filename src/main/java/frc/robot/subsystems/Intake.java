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
import frc.robot.util.CANSparkMaxCurrent;

public class Intake extends SubsystemBase {

    // public static RGB s_RGB = new RGB();

    private CANSparkMaxCurrent intakeMotor;
    private boolean gamePiece;
    private DigitalInput beamBreak;

    public Intake() {
        beamBreak = new DigitalInput(Constants.Intake.kIntakeBeamBreakDIOPin);
        gamePiece = hasGamePiece();

        intakeMotor = new CANSparkMaxCurrent(Constants.Intake.INTAKE, MotorType.kBrushed);

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
        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
        // if(DriverStation.isTeleopEnabled()){
        //     if(gamePiece != hasGamePiece() && hasGamePiece()){
        //         RobotContainer.s_RGB.setLED(State.ORANGESOLID);
        //     } else if (gamePiece != hasGamePiece() && !hasGamePiece()){
        //         RobotContainer.s_RGB.setLED(State.ORANGEBLINK);
        //     }
        // }
        // gamePiece = hasGamePiece();
    }

    public boolean hasGamePiece() {
        return !beamBreak.get();
    }

    public void intakeStop() {
        DriverStation.reportWarning("Intake stop", false);
        intakeMotor.set(0);
    }

    public void intakeForward() {
        DriverStation.reportWarning("Intake forward", false);
        intakeMotor.set(1.0);
    }

    public void intakeReverse() {
        DriverStation.reportWarning("Intake reverse", false);
        intakeMotor.set(-1.0);
    }
}
