// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);


  /* Drive Controls */
  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Set Default commands for subsystems
    setDefaultCommands();
  }

  private void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    driver.x().onTrue(new InstantCommand(() -> s_Swerve.resetAllModulestoAbsol()));
  }

  private void setDefaultCommands() {
    s_Swerve.setDefaultCommand(new SwerveTeleCMD(
      s_Swerve,
      () -> driver.getRawAxis(translationAxis),
      () -> driver.getRawAxis(strafeAxis),
      () -> -driver.getRawAxis(rotationAxis),
      () -> driver.povDown().getAsBoolean(),
      () -> driver.leftBumper().getAsBoolean(),
      () -> driver.rightBumper().getAsBoolean()
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup();
  }
}
