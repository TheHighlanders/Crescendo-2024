// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Notes;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveMoveToCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.commands.deployIntakeCMD;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceSideAutonCMDG extends SequentialCommandGroup {

    /** Creates a new AmpSideAutonCMDG. */
    public SourceSideAutonCMDG(Swerve swerve, Intake intake, Pivot pivot, Shooter shooter, Localizer localizer) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
            // RobotContainer.autonShootRoutineCMDG,
            new alignShootCMDG(shooter, intake, pivot, swerve, localizer, () -> RobotContainer.s_Localizer.getDistanceToSpeaker()),
            new PrintCommand("Post Shoot"),
            new WaitCommand(3),
            new deployIntakeCMD(pivot, intake, false),
            new PrintCommand("Post Deploy"),
            new WaitUntilCommand(() -> pivot.intakeAtSetpointGround()),
            new InstantCommand(() -> intake.intakeForward()),
            new ParallelCommandGroup(
                SwerveMoveToCMD.getAutoPath(swerve, new Pose2d(Notes.SourceClose, new Rotation2d(Math.PI))),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> intake.hasGamePiece()),
                    new PrintCommand("Had Game Piece"),
                    new InstantCommand(() -> intake.intakeStop()),
                    pivot.retractIntake(),
                    new WaitUntilCommand(() -> pivot.intakeAtSetpointShooter()),
                    new PrintCommand("Retracted")
                )
            ),
            new PrintCommand("Post Move PCMDG"),
            new WaitCommand(1),
            new alignShootCMDG(shooter, intake, pivot, swerve, localizer, () -> RobotContainer.s_Localizer.getDistanceToSpeaker())
        );
    }
}
