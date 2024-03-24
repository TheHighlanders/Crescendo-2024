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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConsts;
import frc.robot.auton.AmpSideAutonCMDG;
import frc.robot.auton.MidSideAutonCMDG;
import frc.robot.auton.ShootAndLeaveMidAutonCMDG;
import frc.robot.auton.ShootAndLeaveSourceSideAutonCMDG;
import frc.robot.auton.ShootAutonCMDG;
import frc.robot.auton.SourceSideAutonCMDG;
import frc.robot.commands.SwerveMoveToCMD;
import frc.robot.commands.SwerveTeleCMD;
import frc.robot.commands.alignShootCMDG;
import frc.robot.commands.deployIntakeCMD;
import frc.robot.commands.runIntakeCMD;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.RGB.State;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.function.BooleanSupplier;
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

    public static DoubleSupplier distToSpeaker = s_Localizer::getDistanceToSpeaker;
    /* Auton */
    private SendableChooser<Command> autoChooser;

    /* Triggers */
    public static Trigger hasGamePiece = new Trigger(s_Intake::hasGamePiece);

    /* Commands */

    static Runnable nop = () -> {};
    static BooleanSupplier falseSupplier = () -> false;
    static BooleanSupplier trueSupplier = () -> true;
    static Consumer<Boolean> emptyConsumable = t -> {};

    public static Command alignRobot = new SwerveMoveToCMD(s_Swerve, s_Localizer::getAngleToSpeaker).withName("alignRobotManual");

    public static Command alignPivot = new FunctionalCommand(
        () ->
            s_Pivot.alignPivot(() -> {
                try {
                    return s_Pivot.interpolate(distToSpeaker).getArmExtension();
                } catch (Exception e) {
                    DriverStation.reportWarning("Unable to retreve Arm Extension" + e.toString(), false);
                    return 16;
                }
            }),
        nop,
        i -> DriverStation.reportWarning("Align Pivot End " + i, false),
        trueSupplier,
        s_Pivot
    )
        .withName("alignPivot");

    public static Command driveShooterAngle = new FunctionalCommand(
        nop,
        () -> s_Pivot.driveShooterAngleManual(operator.getRightY() * -0.4),
        emptyConsumable,
        falseSupplier,
        s_Pivot
    );

    public static Command driveShooterRPM = new FunctionalCommand(
        nop, // Initialize
        () -> s_Shooter.shoot(() -> operator.getLeftY() * -4000), //Execute
        v -> s_Shooter.shootCancel(), // End
        falseSupplier,
        s_Shooter // Requirements
    );

    public static Command readyPositions = new InstantCommand(s_Pivot::readyPositions);
    public static Command deployIntake = new deployIntakeCMD(s_Pivot, s_Intake, false);
    public static Command retractIntake = new deployIntakeCMD(s_Pivot, s_Intake, true);
    public static Command ampPosIntake = new InstantCommand(() -> s_Pivot.alignIntakeToAmp());
    public static Command runIntakeOutCMD = new StartEndCommand(s_Intake::intakeReverse, s_Intake::intakeStop, s_Intake);
    public static Command intakeHoldPos = new InstantCommand(s_Pivot::shooterAngleHold);
    public static Command intakeRetractAndSuck = new ParallelDeadlineGroup(
        new runIntakeCMD(s_Intake, true).withTimeout(0.5),
        new deployIntakeCMD(s_Pivot, s_Intake, true)
    );

    public static Command runIntakeInCMD = new StartEndCommand(s_Intake::intakeForward, s_Intake::intakeStop, s_Intake);

    public static Command climbBoth = new StartEndCommand(() -> S_Climber.climbBoth(ClimberConsts.kClimbSpeed), S_Climber::climberStop);

    public static Command resetModules = new InstantCommand(s_Swerve::resetAllModulestoAbsol);
    public static Command zeroGyroCommand = new InstantCommand(s_Swerve::zeroGyro);
    public static Command armClimbPos = new InstantCommand(() -> s_Pivot.alignPivot(18.75));
    public static Command idleShooterCMD = new InstantCommand(s_Shooter::shootIdle);

    public static Command orangeRGB = new InstantCommand(() -> s_RGB.setLED(State.ORANGESOLID));
    public static Command orangeRBGblink = new InstantCommand(() -> s_RGB.setLED(State.ORANGEBLINK));

    public static Command autonShootRoutineCMDG = new alignShootCMDG(
        s_Shooter,
        s_Intake,
        s_Pivot,
        s_Swerve,
        s_Localizer,
        () -> s_Localizer.getDistanceToSpeaker()
    );

    public RobotContainer() {
        configureAuton();
        setDefaultCommands();
        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        hasGamePiece
            .onTrue(idleShooterCMD.andThen(intakeRetractAndSuck).alongWith(setRumble(1).withTimeout(0.5)).alongWith(orangeRGB))
            .onFalse(orangeRBGblink);
        
        driver.y().onTrue(zeroGyroCommand);

        // s_RGB.setLED(State.ORANGESOLID) change this on hasGamePiece state change

        /* Intake Button Bindings */
        driver.rightTrigger(0.75).whileTrue(runIntakeOutCMD); // Runs intake out, alt new runIntakeCMD(s_Intake, false)
        driver.rightBumper().whileTrue(runIntakeInCMD); // Runs intake in, alt new runIntakeCMD(s_Intake, true)
        driver.a().onTrue(deployIntake);
        driver.x().onTrue(retractIntake);
        driver.b().onTrue(ampPosIntake);
        driver
            .start()
            .onTrue(
                new InstantCommand(() -> {
                    s_RGB.setLED(RGB.State.BROWN);
                })
            ); // Toggles Poop Mode

        /* Shooter Button Bindings */
        operator.y().and(hasGamePiece).whileTrue(autonShootRoutineCMDG);
        operator.y().and(hasGamePiece.negate()).whileTrue(setRumble(0.5));

        operator.rightStick().whileTrue(driveShooterAngle);
        operator.rightStick().onFalse(intakeHoldPos); // Manual Pivot Angle Control
        operator
            .rightTrigger(0.1) //Only runs when Trigger depressed above 0.1
            .whileTrue(driveShooterRPM);

        operator.povDown().onTrue(resetModules);
        operator.leftBumper().debounce(0.1).whileTrue(climbBoth);
        operator.rightBumper().whileTrue(runIntakeOutCMD);
        operator.a().onTrue(armClimbPos);
        operator.b().onTrue(alignPivot);
        operator.x().onTrue(readyPositions);
        operator
            .start()
            .toggleOnTrue(new InstantCommand(() -> hasGamePiece = new Trigger(trueSupplier)))
            .toggleOnFalse(new InstantCommand(() -> hasGamePiece = new Trigger(s_Intake::hasGamePiece)));
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

    public static Command setRumble(double value) {
        return new StartEndCommand(
            () -> {
                operator.getHID().setRumble(RumbleType.kBothRumble, value);
                driver.getHID().setRumble(RumbleType.kBothRumble, value);
            },
            () -> {
                operator.getHID().setRumble(RumbleType.kBothRumble, 0);
                driver.getHID().setRumble(RumbleType.kBothRumble, 0);
            }
        );
    }
}
