// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: this command is simple enough to be inline
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class deployIntakeCMD extends Command {

    /** Creates a new deployIntakeCMD. */
    Pivot pivot;
    boolean desiredState;
    Intake intake;

    /**
     * 
     * @param pivot
     * @param desiredState true = align to shooter, false = align to ground
     */
    public deployIntakeCMD(Pivot pivot, Intake intake, boolean desiredState) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.pivot = pivot;
        this.intake = intake;
        this.desiredState = desiredState;
        addRequirements(pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (desiredState) {
            pivot.alignIntakeToShooter();
            intake.intakeForward();
        } else {
            pivot.alignIntakeToGround();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
