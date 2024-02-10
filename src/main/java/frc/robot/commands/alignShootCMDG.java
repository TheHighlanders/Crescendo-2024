// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.util.InterpolatableShotData;
import java.util.function.Consumer;

public class alignShootCMDG extends SequentialCommandGroup {

    private final Shooter m_shooter;
    public final Intake m_intake;
    public final Pivot m_Pivot;
    public final Swerve m_Swerve;
    public final Localizer m_Localizer;

    Runnable emptyRunnable = () -> {
    };

    Consumer<Boolean> emptyConsumable = t -> {
    };

    public alignShootCMDG(
            Shooter shoot,
            Intake intake,
            Pivot pivot,
            Swerve swerve,
            Localizer localizer) {
        m_shooter = shoot;
        m_intake = intake;
        m_Pivot = pivot;
        m_Swerve = swerve;
        m_Localizer = localizer;

        InterpolatableShotData currentShotData = m_Pivot.interpolate(
                m_Localizer.getDistanceToSpeaker());
        // this should resolve before we start rotating hopefully

        addCommands(
                new ParallelCommandGroup(
                        new FunctionalCommand(
                                () -> m_Pivot.alignPivot(currentShotData::getArmAngle),
                                emptyRunnable::run,
                                emptyConsumable,
                                m_Pivot::atSetpoints),
                        new SwerveMoveToCMD(m_Swerve, () -> localizer.getAngleToSpeaker())),
                new ParallelRaceGroup(
                        new StartEndCommand(m_shooter::shoot, m_shooter::shootCancel),
                        new FunctionalCommand(
                                m_intake::intakeStartout,
                                emptyRunnable::run,
                                v -> m_intake.intakeStop(),
                                m_shooter::hasGamePiece)));
    }
}
