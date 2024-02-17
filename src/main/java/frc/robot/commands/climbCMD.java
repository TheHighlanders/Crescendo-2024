// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConsts;
import frc.robot.subsystems.Climber;

public class climbCMD extends Command {
  /** Creates a new climbCMD. */
  Climber climber;
  boolean left;
  public climbCMD(boolean left, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.left = left;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(left){
      climber.climbLeft(ClimberConsts.kClimbSpeed);
    } else{
      climber.climbRight(ClimberConsts.kClimbSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
