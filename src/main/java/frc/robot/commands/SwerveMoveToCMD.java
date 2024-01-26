// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveMoveToCMD extends Command {
  private Swerve s_Swerve;

  PIDController xPID;
  PIDController yPID;
  PIDController aPID;

  double endX;
  double endY;
  double endAngle;
  
  /** Creates a new SwerveMoveToCMD. */
  public SwerveMoveToCMD(Swerve s_Swerve, Pose2d target) {
    this.s_Swerve = s_Swerve;
    endX = target.getX();
    endY = target.getY();
    endAngle = target.getRotation().getDegrees();

    xPID = new PIDController(5, 0.1, 0.13);
    yPID = new PIDController(5, 0.1, 0.13);
    aPID = new PIDController(5, 0, 0.13);

    xPID.setIntegratorRange(-100,100);
    yPID.setIntegratorRange(-100,100);

    xPID.setTolerance(Constants.SwerveMoveConsts.xDeadzone);
    yPID.setTolerance(Constants.SwerveMoveConsts.yDeadzone);

    aPID.setTolerance(Constants.SwerveMoveConsts.aDeadzone);
    aPID.enableContinuousInput(0, 360);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPID.setSetpoint(endX);
    yPID.setSetpoint(endY);
    aPID.setSetpoint(endAngle);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("A PID ERROR", aPID.getPositionError());
    SmartDashboard.putNumber("X PID ERROR", xPID.getPositionError());
    SmartDashboard.putNumber("Y PID ERROR", yPID.getPositionError());

    double aCalc = -aPID.calculate(s_Swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("A CALC", aCalc);

    s_Swerve.drive(
      new Translation2d( xPID.calculate(s_Swerve.getPose().getX()), yPID.calculate(s_Swerve.getPose().getY() )),
      Rotation2d.fromDegrees(aCalc),
      true, 
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("POINT MOVE ENDED", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() &&  aPID.atSetpoint();
  }
}
