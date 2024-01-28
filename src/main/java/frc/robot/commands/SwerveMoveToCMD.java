// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveMoveToCMD extends Command {

    private Swerve s_Swerve;

    PIDController xPID;
    PIDController yPID;
    PIDController aPID;

    double endX;
    double endY;
    double endAngle;

    /** Creates a new SwerveMoveToCMD. */
    public SwerveMoveToCMD(Swerve s_Swerve, double X, double Y, double angleDeg) {
        this.s_Swerve = s_Swerve;
        endX = X;
        endY = Y;
        endAngle = angleDeg;

        xPID = new PIDController(0, 1, 0);
        yPID = new PIDController(0, 1, 0);
        aPID = new PIDController(0.5, 0.05, 0);

        aPID.setTolerance(7);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xPID.setSetpoint(endX);
        yPID.setSetpoint(endY);
        aPID.setSetpoint(endAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Swerve.drive(
            new Translation2d(
                xPID.calculate(s_Swerve.getPose().getX()),
                yPID.calculate(s_Swerve.getPose().getY())
            ),
            Rotation2d.fromDegrees(aPID.calculate(s_Swerve.getPose().getRotation().getDegrees())),
            true,
            false
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return /*xPID.atSetpoint() && yPID.atSetpoint() &&*/aPID.atSetpoint();
    }
}
