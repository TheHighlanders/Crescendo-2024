// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.util.InterpolatableShotData;

public class alignShootCMDG extends SequentialCommandGroup {

    private final Shooter m_shooter;
    public final intake m_intake;
    public final Pivot m_Pivot;
    public final Swerve m_Swerve;
    public final Localizer m_Localizer;

    public alignShootCMDG(
        Shooter shoot,
        intake intake,
        Pivot pivot,
        Swerve swerve,
        Localizer localizer
    ) {
        m_shooter = shoot;
        m_intake = intake;
        m_Pivot = pivot;
        m_Swerve = swerve;
        m_Localizer = localizer;

        InterpolatableShotData currentShotData = m_Pivot.interpolate(
            m_Localizer.getDistanceToSpeaker()
        );
        //this should resolve before we start rotating hopefully
        double angleToSpeaker = localizer.getAngleToSpeaker();

        addCommands(
            new InstantCommand(() -> {
                DriverStation.reportWarning("desired arm angle" + Double.toString(currentShotData.getArmAngle()) + ": angle to speaker" + Double.toString(angleToSpeaker), false);
            })
            // new ParallelCommandGroup(
            //     new FunctionalCommand(
            //         () -> m_Pivot.alignPivot(currentShotData::getArmAngle),
            //         () -> {},
            //         value -> {},
            //         m_Pivot::atSetpoints
            //     ),
            //     new SwerveMoveToCMD(m_Swerve, Double.NaN, Double.NaN, angleToSpeaker)
            // ),
            // new ParallelRaceGroup(
            //     new StartEndCommand(m_shooter::shoot, m_shooter::shootCancel),
            //     new FunctionalCommand(
            //         m_intake::intakeStartout,
            //         () -> {},
            //         value -> m_intake.intakeStop(),
            //         m_shooter::hasGamePiece
            //     )
            // )
        );
    }
}
