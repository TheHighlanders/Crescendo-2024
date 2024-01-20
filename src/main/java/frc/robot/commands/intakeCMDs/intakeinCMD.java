// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeinCMD extends Command {
  /** Creates a new intakeinCMD. */
  public final intake intakein;

  public intakeinCMD(intake subIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakein = subIntake;
    addRequirements(intakein);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakein.intakeStarting();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakein.intakestop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
