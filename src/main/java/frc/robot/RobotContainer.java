// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LEDloadingBarCMD;
import frc.robot.commands.SwerveMoveToCMD;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.commands.climbCMD;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.RGB;
//import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
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
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */
    private static final int translationAxis = XboxController.Axis.kLeftY.value;
    private static final int strafeAxis = XboxController.Axis.kLeftX.value;
    private static final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    int i = 100;
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision s_Vision = new Vision();
    public static final Climber S_Climber = new Climber();

    public static final Localizer s_Localizer = new Localizer(s_Swerve, s_Vision);
    public static final Supplier<Pose2d> getLocalizedPose = () -> s_Localizer.getPose();
    public static final Consumer<Pose2d> resetLocalizedPose = (Pose2d pose) -> s_Localizer.resetOdoPose2d(pose);
    public RGB s_RGB = new RGB();

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
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driver.x().onTrue(new InstantCommand(() -> s_Swerve.resetAllModulestoAbsol()));

        // driver.b().onTrue(new SwerveMoveToCMD(s_Swerve, () -> s_Localizer.getAngleToSpeaker()));
        driver.b().onTrue(new SwerveMoveToCMD(s_Swerve, new Pose2d(0.5, -1, Rotation2d.fromDegrees(180))));
        //driver.b().onTrue(new TestMove(s_Swerve));

        driver.a().onTrue(new LEDloadingBarCMD(s_RGB));

        /* Climber Button Bindings */
        operator.leftBumper().onTrue(new climbCMD(true, S_Climber));
        operator.rightBumper().onTrue(new climbCMD(false, S_Climber));
    }

    private void configureAuton() {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
            new SwerveTeleCMD(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> driver.povDown().getAsBoolean(),
                () -> driver.leftBumper().getAsBoolean()
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
