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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Shooter;
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
    public static final Shooter s_Shooter = new Shooter();
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision s_Vision = new Vision();
    public static final Pivot s_Pivot = new Pivot();
    public static final Intake s_Intake = new Intake();

    public static final Localizer s_Localizer = new Localizer(s_Swerve, s_Vision);
    public static final Supplier<Pose2d> getLocalizedPose = () -> s_Localizer.getPose();
    public static final Consumer<Pose2d> resetLocalizedPose = (Pose2d pose) -> s_Localizer.resetOdoPose2d(pose);
    public static RGB s_RGB = new RGB();

    /* Auton */
    private SendableChooser<Command> autoChooser;

    public static Command deployIntakeCMD = new InstantCommand(
        () -> {
            if (s_Pivot.getIntakeDeploy()) {
                s_Pivot.intakeIn();
            } else {
                s_Pivot.intakeOut();
            }
        },
        s_Pivot
    );

    public static Command runIntakeOutCMD = new StartEndCommand(s_Intake::intakeReverse, s_Intake::intakeStop, s_Intake);
    public static Command runIntakeInCMD = new StartEndCommand(s_Intake::intakeForward, s_Intake::intakeStop, s_Intake);
    public static Command gamePieceOverrideCMD = new InstantCommand(s_Intake::gamePieceDetectionOverride);
    public static Command readyPositionsCMD = new InstantCommand(s_Pivot::readyPositions);

    public static alignShootCMDG autonShootRoutineCMDG = new alignShootCMDG(s_Shooter, s_Intake, s_Pivot, s_Swerve, s_Localizer);

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

        /* Intake Button Bindings */
        driver.start().whileTrue(runIntakeOutCMD); // Runs intake out, alt new runIntakeCMD(s_Intake, false)
        driver.rightBumper().whileTrue(runIntakeInCMD); // Runs intake in, alt new runIntakeCMD(s_Intake, true)
        driver.leftBumper().whileTrue(new frc.robot.commands.deployIntakeCMD(s_Pivot));
        operator.x().onTrue(gamePieceOverrideCMD);
        operator.back().onTrue(readyPositionsCMD);

        /* Shooter Button Bindings */
        operator.y().whileTrue(autonShootRoutineCMDG); // Automatic shooting routine
        operator.rightStick().whileTrue(new InstantCommand(() -> s_Pivot.driveShooterAngleManual(operator.getRightY() * -0.25))); // Manual Pivot Angle Control
        operator/* .whileTrue(new InstantCommand(() -> s_Shooter.shoot(() -> 1)));*/
            .rightTrigger(0.1) //Only runs when Trigger depressed above 0.1
            .whileTrue(
                new FunctionalCommand(
                    () -> {}, // Initialize
                    () -> {
                        s_Shooter.shoot(() -> operator.getRightTriggerAxis() * 1000); //Execute
                    },
                    v -> {
                        s_Shooter.shootCancel(); // End
                    },
                    () -> {
                        return false; // Is Finished
                    },
                    s_Shooter // Requirements
                )
            );
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

        // s_Pivot.setDefaultCommand(
        //     new FunctionalCommand(
        //         () -> {}, // Initialize
        //         () -> {
        //             s_Pivot.driveShooterAngleManual(() -> operator.getRawAxis(translationAxis) / 10); //Execute
        //         },
        //         v -> {
        //             s_Pivot.stopShooterAngle(); // End
        //         },
        //         () -> {
        //             return false; // Is Finished
        //         },
        //         s_Shooter // Requirements
        //     )
        // );
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
