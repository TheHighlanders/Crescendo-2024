// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.util.InterpolatableShotData;
import frc.robot.util.InterpolatingShotTreeMapContainer;

public class alignShootCMDG extends SequentialCommandGroup {

  private InterpolatingShotTreeMapContainer iTreeMapContainer = new InterpolatingShotTreeMapContainer();

  private final Shooter m_shooter;
  public final intake m_intake;
  public final Pivot m_Pivot;
  public final Swerve m_Swerve;

  public alignShootCMDG(
    Shooter shoot,
    intake intake,
    Pivot pivot,
    Swerve swerve
  ) {
    m_shooter = shoot;
    m_intake = intake;
    m_Pivot = pivot;
    m_Swerve = swerve;

    // TODO: get from vision
    InterpolatableShotData currentShotData = iTreeMapContainer.interpolate(1);

    addCommands(
      new ParallelCommandGroup(
        new FunctionalCommand(
          () -> m_Pivot.alignPivot(currentShotData::getArmAngle),
          () -> {},
          value -> {},
          m_Pivot::atSetpoints
        ),
        new SwerveMoveToCMD(m_Swerve, -1, -1, 0)
      ),
      new ParallelRaceGroup(
        new StartEndCommand(m_shooter::shoot, m_shooter::shootCancel),
        new FunctionalCommand(
          m_intake::intakeStartout,
          () -> {},
          value -> m_intake.intakeStop(),
          m_shooter::hasGamePiece
        )
      )
    );
  }
}
