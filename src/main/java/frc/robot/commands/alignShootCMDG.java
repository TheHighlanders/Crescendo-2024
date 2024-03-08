// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.InterpolatableShotData;
import java.util.function.Consumer;

public class alignShootCMDG extends SequentialCommandGroup {

    private final Shooter m_shooter;
    public final Intake m_intake;
    public final Pivot m_Pivot;
    public final Swerve m_Swerve;
    public final Localizer m_Localizer;

    Runnable emptyRunnable = () -> {};

    Consumer<Boolean> emptyConsumable = t -> {};

    public alignShootCMDG(Shooter shoot, Intake intake, Pivot pivot, Swerve swerve, Localizer localizer) {
        m_shooter = shoot;
        m_intake = intake;
        m_Pivot = pivot;
        m_Swerve = swerve;
        m_Localizer = localizer;

        InterpolatableShotData currentShotData = m_Pivot.interpolate(m_Localizer.getDistanceToSpeaker());
        // this should resolve before we start rotating hopefully

        addCommands(
            // move swere to face speaker and align pivot
            new ParallelCommandGroup(
                // align pivot, is finished when asSetpoints returns true
                new InstantCommand(() -> m_Pivot.alignPivot(currentShotData::getArmExtension)),
                // runs the swerve move to command to the angle of the speaker
                new SwerveMoveToCMD(m_Swerve, localizer::getAngleToSpeaker),
                new WaitUntilCommand(m_Pivot::atSetpointsAtShooter)
            ),
            // runs the intake and the shooter for 3 seconds and then stops them when the time us up
            new ParallelDeadlineGroup(
                new ParallelRaceGroup(
                    new WaitUntilCommand(() -> !(shoot.getBeamBreak() || intake.hasGamePiece())).andThen(new WaitCommand(0.5)),
                    new WaitCommand(Constants.Shooter.kWaitTimeBeforeStop)
                        .andThen(new PrintCommand("Override is " + intake.gamePieceDetectionOverride()))
                ),
                new StartEndCommand(() -> m_shooter.shoot(currentShotData::getRPM), m_shooter::shootCancel),
                new StartEndCommand(m_intake::intakeReverse, m_intake::intakeStop)
            )
        );
    }
}
