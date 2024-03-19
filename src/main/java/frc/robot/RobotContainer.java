// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConsts;
import frc.robot.auton.*;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.commands.deployIntakeCMD;
import frc.robot.commands.runIntakeCMD;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
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

    public static final double speedMult = 0.5;
    public static DoubleSupplier distToSpeaker;

    /* Controllers */
    private static final CommandXboxController driver = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */
    private static final int translationAxis = XboxController.Axis.kLeftY.value;
    private static final int strafeAxis = XboxController.Axis.kLeftX.value;
    private static final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    public static final Shooter s_Shooter = new Shooter();
    public static final Swerve s_Swerve = new Swerve();
    public static final Vision s_Vision = new Vision();
    public static final Climber S_Climber = new Climber();
    public static final Pivot s_Pivot = new Pivot();
    public static final Intake s_Intake = new Intake();

    public static final Localizer s_Localizer = new Localizer(s_Swerve, s_Vision);
    public static final Supplier<Pose2d> getLocalizedPose = s_Localizer.getPose();
    public static final Consumer<Pose2d> resetLocalizedPose = (Pose2d pose) -> s_Localizer.resetOdoPose2d(pose);
    public static RGB s_RGB = new RGB();

    /* Auton */
    private SendableChooser<Command> autoChooser;

    /* Commands */
    public static Command driveShooterAngle = new FunctionalCommand(
        () -> {},
        () -> {
            s_Pivot.driveShooterAngleManual(operator.getRightY() * -0.5);
        },
        v -> {},
        () -> {
            return false;
        },
        s_Pivot
    );

    public static Command driveShooterRPM = new FunctionalCommand(
        () -> {}, // Initialize
        () -> {
            s_Shooter.shoot(() -> operator.getLeftY() * -3000); //Execute
        },
        v -> {
            s_Shooter.shootCancel(); // End
        },
        () -> {
            return false; // Is Finished
        },
        s_Shooter // Requirements
    );

    public static Command deployIntake = new deployIntakeCMD(s_Pivot, s_Intake, false);
    public static Command runIntakeOutCMD = new StartEndCommand(s_Intake::intakeReverse, s_Intake::intakeStop, s_Intake);
    public static Command intakeFloorCommand = new InstantCommand(s_Pivot::alignIntakeToGround);
    public static Command intakeShooterCommand = new InstantCommand(s_Pivot::alignIntakeToShooter).andThen(setRumble(1, 0.2, false, false));
    public static Command intakeHoldPos = new InstantCommand(() -> s_Pivot.shooterAngleHold());
    public static Command intakeRetract = new ParallelDeadlineGroup(
        new WaitCommand(0.75),
        setRumble(1, 0.5, true, true),
        new SequentialCommandGroup(new WaitCommand(0.5), new runIntakeCMD(s_Intake, true)),
        new deployIntakeCMD(s_Pivot, s_Intake, true)
    );

    public static Command runIntakeInCMD = new StartEndCommand(s_Intake::intakeForward, s_Intake::intakeStop, s_Intake);
    public static Command climbLeftSoft = new StartEndCommand(
        () -> S_Climber.climbLeft(ClimberConsts.kClimbSpeed * speedMult),
        () -> S_Climber.climbLeft(0)
    );
    public static Command climbRightSoft = new StartEndCommand(
        () -> S_Climber.climbRight(ClimberConsts.kClimbSpeed * speedMult),
        () -> S_Climber.climbRight(0)
    );
    public static Command climbRight = new StartEndCommand(() -> S_Climber.climbRight(ClimberConsts.kClimbSpeed), () -> S_Climber.climbRight(0));
    public static Command climbLeft = new StartEndCommand(() -> S_Climber.climbLeft(ClimberConsts.kClimbSpeed), () -> S_Climber.climbLeft(0));

    public static Command resetModules = new InstantCommand(() -> s_Swerve.resetAllModulestoAbsol());
    public static Command zeroGyroCommand = new InstantCommand(() -> s_Swerve.zeroGyro());

    public static alignShootCMDG autonShootRoutineCMDG = new alignShootCMDG(
        s_Shooter,
        s_Intake,
        s_Pivot,
        s_Swerve,
        s_Localizer,
        () -> s_Localizer.getDistanceToSpeaker()
    );

    public RobotContainer() {
        s_RGB.changeString("7");
        configureBindings();
        configureAuton();
        setDefaultCommands();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        driver.y().onTrue(zeroGyroCommand);

        /* Intake Button Bindings */
        driver.rightTrigger(0.75).whileTrue(runIntakeOutCMD); // Runs intake out, alt new runIntakeCMD(s_Intake, false)
        driver.rightBumper().whileTrue(runIntakeInCMD); // Runs intake in, alt new runIntakeCMD(s_Intake, true)
        driver.a().onTrue(deployIntake);
        driver.x().onTrue(intakeRetract);

        /* Shooter Button Bindings */
        operator.y().whileTrue(autonShootRoutineCMDG); // Automatic shooting routine
        operator.rightStick().whileTrue(driveShooterAngle);
        operator.rightStick().onFalse(intakeHoldPos); // Manual Pivot Angle Control
        operator
            .rightTrigger(0.1) //Only runs when Trigger depressed above 0.1
            .whileTrue(driveShooterRPM);
        operator.back().whileTrue(climbLeftSoft);
        operator.start().whileTrue(climbRightSoft);
        operator.povDown().onTrue(resetModules);
        operator.leftBumper().whileTrue(climbLeft);
        operator.rightBumper().and(operator.rightTrigger(0.1).negate()).whileTrue(climbRight);
        operator.rightBumper().and(operator.rightTrigger(0.1)).whileTrue(runIntakeOutCMD);
    }

    private void configureAuton() {
        // autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Mid 2Piece", new MidSideAutonCMDG(s_Swerve, s_Intake, s_Pivot, s_Shooter, s_Localizer));
        autoChooser.addOption("Amp 2Piece", new AmpSideAutonCMDG(s_Swerve, s_Intake, s_Pivot, s_Shooter, s_Localizer));
        autoChooser.addOption("Source 2Piece", new SourceSideAutonCMDG(s_Swerve, s_Intake, s_Pivot, s_Shooter, s_Localizer));
        autoChooser.addOption("Just Shoot", new ShootAutonCMDG(s_Swerve, s_Intake, s_Pivot, s_Shooter, s_Localizer));
        autoChooser.addOption("None", new SequentialCommandGroup());
        autoChooser.addOption("Source Side 1PLeave", new ShootAndLeaveSourceSideAutonCMDG(s_Shooter, s_Intake, s_Swerve, s_Pivot, s_Localizer));
        autoChooser.addOption("Mid 1P Leave", new ShootAndLeaveMidAutonCMDG(s_Shooter, s_Intake, s_Swerve, s_Pivot, s_Localizer));

        autoChooser.setDefaultOption("Just Shoot", new ShootAutonCMDG(s_Swerve, s_Intake, s_Pivot, s_Shooter, s_Localizer));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
            new SwerveTeleCMD(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
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
        return autoChooser.getSelected();
    }

    public static Command setRumble(double value, double length, boolean right, boolean driverController) {
        RumbleType rumbleSide = right ? RumbleType.kRightRumble : RumbleType.kLeftRumble;
        if (driverController) {
            return new InstantCommand(() -> {
                driver.getHID().setRumble(rumbleSide, value);
            })
                .andThen(new WaitCommand(length))
                .andThen(
                    new InstantCommand(() -> {
                        driver.getHID().setRumble(rumbleSide, 0);
                    })
                );
        } else {
            return new InstantCommand(() -> {
                operator.getHID().setRumble(rumbleSide, value);
            })
                .andThen(new WaitCommand(length))
                .andThen(
                    new InstantCommand(() -> {
                        operator.getHID().setRumble(rumbleSide, 0);
                    })
                );
        }
    }
}
