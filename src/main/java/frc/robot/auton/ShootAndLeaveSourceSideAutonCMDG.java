// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Points;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveMoveToCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndLeaveSourceSideAutonCMDG extends SequentialCommandGroup {

    /** Creates a new ShootAndLeaveSourceSideAutonCMDG. */
    public ShootAndLeaveSourceSideAutonCMDG(Shooter shooter, Intake intake, Swerve swerve, Pivot pivot, Localizer localizer) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new alignShootCMDG(shooter, intake, pivot, swerve, localizer, () -> RobotContainer.s_Localizer.getDistanceToSpeaker()),
            SwerveMoveToCMD.getAutoPath(swerve, new Pose2d(Points.shootAndLeaveS1, new Rotation2d(Math.PI))),
            //TODO: Uncomment
            SwerveMoveToCMD.getAutoPath(swerve, new Pose2d(Points.shootAndLeaveS2, new Rotation2d(Math.PI)))
        );
    }
}
