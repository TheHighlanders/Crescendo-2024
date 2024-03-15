// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConst;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class SwerveMoveToCMD extends Command {

    private Swerve s_Swerve;

    PIDController xPID;
    PIDController yPID;
    PIDController aPID;

    Supplier<Pose2d> targetSup;

    double endX;
    double endY;
    double endAngle;

    boolean translate;

    static double fieldLine = Units.feetToMeters(54) + Units.inchesToMeters(3.25);

    /** Creates a new SwerveMoveToCMD. */
    public SwerveMoveToCMD(Swerve s_Swerve, Supplier<Pose2d> target, boolean translate) {
        this.s_Swerve = s_Swerve;
        this.translate = translate;

        // endX = target.get().getX();
        // endY = target.get().getY();
        // endAngle = target.get().getRotation().getDegrees();

        this.targetSup = target;

        xPID = new PIDController(SwerveConst.kTranslateP, SwerveConst.kTranslateI, SwerveConst.kTranslateD);
        yPID = new PIDController(SwerveConst.kTranslateP, SwerveConst.kTranslateI, SwerveConst.kTranslateD);

        aPID = new PIDController(SwerveConst.kRotateP, SwerveConst.kRotateI, SwerveConst.kRotateD);

        xPID.setIntegratorRange(-100, 100);
        yPID.setIntegratorRange(-100, 100);

        xPID.setTolerance(Constants.SwerveMoveConsts.posPosTolerance, Constants.SwerveMoveConsts.posVelTolerance);
        yPID.setTolerance(Constants.SwerveMoveConsts.posPosTolerance, Constants.SwerveMoveConsts.posVelTolerance);

        aPID.setTolerance(Constants.SwerveMoveConsts.aPosTolerance/*, Constants.SwerveMoveConsts.aVelTolerance*/);
        aPID.enableContinuousInput(0, 360);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve);
    }

    public SwerveMoveToCMD(Swerve s_Swerve, Supplier<Rotation2d> heading) {
        this(s_Swerve, () -> new Pose2d(new Translation2d(), heading.get()), false);
    }

    public SwerveMoveToCMD(Swerve s_Swerve, Pose2d pose) {
        this(s_Swerve, () -> pose, true);
    }

    /** Mirrors Pose for Auton */
    public SwerveMoveToCMD(Swerve s_Swerve, Pose2d pose, int i) {
        this(s_Swerve, () -> pose, true);
    }

    public static SwerveMoveToCMD getAutoPath(Swerve swerve, Pose2d pose) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            pose = new Pose2d(fieldLine + (fieldLine - pose.getX()), pose.getY(), pose.getRotation());
        }
        return new SwerveMoveToCMD(swerve, pose);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xPID.setSetpoint(targetSup.get().getX());
        yPID.setSetpoint(targetSup.get().getY());
        aPID.setSetpoint(targetSup.get().getRotation().getDegrees());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double aCalc = -aPID.calculate(s_Swerve.getPose().getRotation().getDegrees());
        if (translate) {
            double xCalc = xPID.calculate(s_Swerve.getPose().getX());
            double yCalc = yPID.calculate(s_Swerve.getPose().getY());
            MathUtil.clamp(xCalc, -1, 1);
            MathUtil.clamp(yCalc, -1, 1);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xCalc, yCalc, aCalc);
            ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, s_Swerve.getYaw());

            s_Swerve.drive(
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
                Rotation2d.fromDegrees(aCalc),
                false,
                true
            );
        } else {
            s_Swerve.drive(new Translation2d(), Rotation2d.fromDegrees(aCalc), true, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(), new Rotation2d(), true, true);
        DriverStation.reportWarning("POINT MOVE ENDED", false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (translate) {
            return xPID.atSetpoint() && yPID.atSetpoint() && aPID.atSetpoint();
        } else {
            return aPID.atSetpoint();
        }
    }
}
