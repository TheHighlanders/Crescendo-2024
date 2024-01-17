// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.SwerveConst;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SlewRateLimiter;

public class PIDweird extends Command {
  private Swerve s_Swerve;

  private DoubleSupplier JoyX;
  private DoubleSupplier JoyY;
  // private BooleanSupplier gridLineUp;

  private SlewRateLimiter translationLimiter;
  private SlewRateLimiter strafeLimiter;

  private PIDController translationController;
  private PIDController rotationController;

  PIDController angularV;

  private enum Speed {
    SLOW,
    NORMAL
  }

  
  public PIDweird(
      Swerve s_Swerve,
      DoubleSupplier JoyX,
      DoubleSupplier JoyY) {
    this.s_Swerve = s_Swerve;
    this.JoyX = JoyX;
    this.JoyY = JoyY;
    // this.gridLineUp = gridLineUp;
    

    angularV = new PIDController(0.004, 0,0.1);
    angularV.setIntegratorRange(-1, 1);
    angularV.enableContinuousInput(-Math.PI, Math.PI);

        

    addRequirements(s_Swerve);
  }

  
  @Override
  public void initialize() {
    
  }
  
  public void execute() {
    double angle = Math.atan2(JoyY.getAsDouble(), JoyX.getAsDouble());
    s_Swerve.drive(
        new Translation2d(),
        Rotation2d.fromRadians(angularV.calculate(s_Swerve.getYaw().getRadians(), angle)),
        true,
        false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
