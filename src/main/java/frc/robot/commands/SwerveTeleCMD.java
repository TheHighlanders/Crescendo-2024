// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConst;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SlewRateLimiter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveTeleCMD extends Command {

    private Swerve s_Swerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftBumper;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;

    private enum Speed {
        SLOW,
        NORMAL,
    }

    public SwerveTeleCMD(
        Swerve s_Swerve,
        DoubleSupplier translationSup,
        DoubleSupplier strafeSup,
        DoubleSupplier rotationSup,
        BooleanSupplier robotCentricSup,
        BooleanSupplier leftBumper
    ) {
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftBumper = leftBumper;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        translationLimiter = new SlewRateLimiter(3.0);
        strafeLimiter = new SlewRateLimiter(3.0);
    }

    public void execute() {
        /* Determining Speeds based on buttons & Ratelimiting */
        Speed speed = leftBumper.getAsBoolean() ? Speed.SLOW : Speed.NORMAL;

        double speedLimit = SwerveConst.speedLimit;
        double angularSpeedLimit = SwerveConst.angularVelocityLimit;

        switch (speed) {
            case SLOW:
                translationLimiter.setRateLimit(SwerveConst.slowAccelerationLimit);
                strafeLimiter.setRateLimit(SwerveConst.slowAccelerationLimit);

                speedLimit = SwerveConst.slowSpeedLimit;
                angularSpeedLimit = SwerveConst.slowAngularVelocityLimit;
            case NORMAL:
                translationLimiter.setRateLimit(SwerveConst.accelerationLimit);
                strafeLimiter.setRateLimit(SwerveConst.accelerationLimit);

                speedLimit = SwerveConst.speedLimit;
                angularSpeedLimit = SwerveConst.angularVelocityLimit;
            default:
                translationLimiter.setRateLimit(SwerveConst.accelerationLimit);
                strafeLimiter.setRateLimit(SwerveConst.accelerationLimit);

                break;
        }

        /* Deadbanding */
        double translationVal;
        double rotationVal;
        double strafeVal = strafeLimiter.calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConst.kStickDeadband));

        rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConst.kStickDeadband);
        translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConst.kStickDeadband);

        s_Swerve.drive(
            new Translation2d(translationLimiter.calculate(translationVal), strafeLimiter.calculate(strafeVal)).times(speedLimit),
            Rotation2d.fromDegrees(rotationVal * (angularSpeedLimit)),
            !robotCentricSup.getAsBoolean(),
            Constants.SwerveConst.kOpenLoop
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
