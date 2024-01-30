// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intake;
import java.util.function.Consumer;
import java.util.function.Supplier;

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
    public static final Shooter s_Shooter = new Shooter();
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision s_Vision = new Vision();
    public static final intake s_Intake = new intake();
    public static final Pivot s_Pivot = new Pivot();

    public static final Localizer s_Localizer = new Localizer(s_Swerve, s_Vision);
    public static final Supplier<Pose2d> getLocalizedPose = () -> s_Localizer.getPose();
    public static final Consumer<Pose2d> resetLocalizedPose = (Pose2d pose) ->
        s_Localizer.resetOdoPose2d(pose);

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
        driver.y().onTrue(new InstantCommand(s_Swerve::zeroGyro));
        driver.a().onTrue(new InstantCommand(s_Swerve::resetAllModulestoAbsol));

        driver.x().onTrue(new alignShootCMDG(s_Shooter, s_Intake, s_Pivot, s_Swerve, s_Localizer));
    }

    private void configureAuton() {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
            new SwerveTeleCMD(
                s_Swerve,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> driver.povDown().getAsBoolean(),
                () -> driver.leftBumper().getAsBoolean(),
                () -> driver.rightBumper().getAsBoolean()
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Uses an Auto to assign a starting position
        //return new PathPlannerAuto("Testing Auton");
        return autoChooser.getSelected();
    }
}
