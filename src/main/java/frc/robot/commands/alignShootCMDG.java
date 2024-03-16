// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.InterpolatableShotData;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class alignShootCMDG extends ParallelCommandGroup {

    private final Shooter m_shooter;
    public final Intake m_intake;
    public final Pivot m_Pivot;
    public final Swerve m_Swerve;
    public final Localizer m_Localizer;
    public DoubleSupplier distance;
    private Supplier<InterpolatableShotData> data;

    Runnable emptyRunnable = () -> {};

    Consumer<Boolean> emptyConsumable = t -> {};

    public static Command startShooter;
    public static Command assignData;
    public static Command alignPivot;
    public static Command alignRobot;
    public static Command waitForGamePiece;
    public static Command outTake;
    public static Command waitForRpmSetpoint;
    public static Command deadline;
    public static Command runIntakeOut;

    public alignShootCMDG(Shooter shoot, Intake intake, Pivot pivot, Swerve swerve, Localizer localizer, DoubleSupplier distance) {
        m_shooter = shoot;
        m_intake = intake;
        m_Pivot = pivot;
        m_Swerve = swerve;
        m_Localizer = localizer;
        this.distance = distance;

        try {
            data = () -> m_Pivot.interpolate(distance.getAsDouble());
        } catch (Exception e) {
            data = () -> m_Pivot.interpolate(1.5);

            DriverStation.reportWarning("Align Data threw", true);
        }

        startShooter =
            new StartEndCommand(
                () -> {
                    DriverStation.reportWarning(data.get() + "", false);
                    m_shooter.shoot(() -> data.get().getRPM() + 75);
                },
                m_shooter::shootCancel
            );

        runIntakeOut = new runIntakeCMD(m_intake, false);
        // align pivot, is finished when asSetpoints returns true
        alignPivot = new InstantCommand(() -> m_Pivot.alignPivot(() -> data.get().getArmExtension()));
        // runs the swerve move to command to the angle of the speaker
        alignRobot = new SwerveMoveToCMD(m_Swerve, localizer::getAngleToSpeaker);
        waitForGamePiece = new WaitUntilCommand(() -> !(!shoot.getBeamBreak() || intake.hasGamePiece())).andThen(new WaitCommand(0.5));
        waitForRpmSetpoint = new WaitUntilCommand(m_shooter::atVelocity).andThen(new PrintCommand("At velocity"));
        deadline = new WaitCommand(Constants.Shooter.kWaitTimeBeforeStop);

        addCommands(
            // alignRobot
            new InstantCommand(() -> {
                m_shooter.shoot(() -> data.get().getRPM() + 75);
            }),
            // alignRobot,
            new SequentialCommandGroup(
                // move swere to face speaker and align pivot
                new ParallelCommandGroup(alignPivot, alignRobot, waitForRpmSetpoint),
                new PrintCommand("In Second Phase"),
                // runs the intake and the shooter for 3 seconds and then stops them when the time us up\
                new ParallelDeadlineGroup(new ParallelRaceGroup(waitForGamePiece, deadline), runIntakeOut),
                new InstantCommand(() -> {
                    m_shooter.shootCancel();
                }),
                new PrintCommand("Ended")
            ),
            new PrintCommand("Automatic Shooter")
        );
    }
}
