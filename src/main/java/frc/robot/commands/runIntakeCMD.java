// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class runIntakeCMD extends Command {

    /** Creates a new runIntakeCMD. */
    Intake m_intake;
    Shooter m_shooter;
    boolean speed;

    public runIntakeCMD(Intake intake, Shooter shooter, boolean forward) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = intake;
        m_shooter = shooter;
        this.speed = forward;

        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (speed) {
            m_intake.intakeForward();
        } else if (m_shooter.aboveMinVelocity()) {
            m_intake.intakeReverse();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("above min velocity", m_shooter.aboveMinVelocity());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.intakeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
