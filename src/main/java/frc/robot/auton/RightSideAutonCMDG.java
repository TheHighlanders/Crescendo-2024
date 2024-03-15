// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Notes;
import frc.robot.commands.SwerveMoveToCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.commands.runIntakeCMD;
import frc.robot.subsystems.*;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSideAutonCMDG extends SequentialCommandGroup {

    public RightSideAutonCMDG(Swerve swerve, Intake intake, Pivot pivot, Shooter shooter, Localizer localizer) {
        addCommands(
            new alignShootCMDG(shooter, intake, pivot, swerve, localizer, localizer::getDistanceToSpeaker),
            new PrintCommand("Post Shoot"),
            new WaitCommand(0.1),
            // new deployIntakeCMD(pivot, intake, false),
            // new PrintCommand("Post Deploy"),
            // new WaitUntilCommand(()->pivot.intakeAtSetpointGround()),

            new ParallelDeadlineGroup(
                SwerveMoveToCMD.getAutoPath(swerve, new Pose2d(Notes.MidClose, new Rotation2d(Math.PI))),
                new runIntakeCMD(intake, shooter, true)
            ),
            new PrintCommand("Post Move"),
            new WaitCommand(1),
            new alignShootCMDG(shooter, intake, pivot, swerve, localizer, localizer::getDistanceToSpeaker)
        );
    }
}
