// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.PIDweird;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.InterpolatableShotData;
import frc.robot.subsystems.Pivot;

import java.sql.Driver;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
    //public final Swerve s_Swerve = new Swerve();
    //public final Shooter s_Shooter = new Shooter();
    public final Pivot s_Pivot = new Pivot();

    /* Auton */
    private SendableChooser<Command> autoChooser;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureAuton();
        // Set Default commands for subsystems
        setDefaultCommands();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        //driver.y().onTrue(new InstantCommand(s_Swerve::zeroGyro));
        //driver.a().onTrue(new InstantCommand(s_Swerve::resetAllModulestoAbsol));

        driver.y().onTrue(new InstantCommand(() -> s_Pivot.driveShooterAngleManual(1)));
        driver.x().onTrue(new InstantCommand(() -> s_Pivot.driveShooterAngleManual(-1)));

        driver.a().onTrue(new InstantCommand(() -> s_Pivot.alignPivot(s_Pivot.interpolate(30)::getArmAngle)));
        driver.b().onTrue(new InstantCommand(() -> s_Pivot.alignPivot(s_Pivot.interpolate(20)::getArmAngle)));

        //driver.x().onTrue(new InstantCommand(s_Shooter::shoot));
        //driver.b().onTrue(new InstantCommand(s_Shooter::shootCancel));
    }

    private void configureAuton() {
        //s_Shooter.shootCancel();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void setDefaultCommands() {
        /*s_Swerve.setDefaultCommand(new SwerveTeleCMD(
        s_Swerve,
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        driver.povDown()::getAsBoolean,
        driver.leftBumper()::getAsBoolean,
        driver.rightBumper()::getAsBoolean));
*/
        // s_Swerve.setDefaultCommand(new PIDweird(s_Swerve, () -> driver.getLeftX(), ()-> driver.getLeftY()));
        s_Pivot.setDefaultCommand(new InstantCommand(() -> s_Pivot.driveShooterAngleManual(() -> driver.getRawAxis(translationAxis))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Uses an Auto to assign a starting position
        return new PathPlannerAuto("New Auto");
        // return autoChooser.getSelected();
    }
}
