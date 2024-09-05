// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConsts;
import frc.robot.subsystems.Climber;
import java.util.function.BooleanSupplier;

public class climbCMD extends Command {

    /** Creates a new climbCMD. */
    Climber climber;
    BooleanSupplier left;
    BooleanSupplier right;

    public climbCMD(BooleanSupplier left, BooleanSupplier right, Climber climber) {
        this.climber = climber;
        this.left = left;
        this.right = right;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean rightButtonDown = right.getAsBoolean();
        boolean leftButtonDown = left.getAsBoolean();

        if (leftButtonDown && rightButtonDown) {
            climber.climbBoth(ClimberConsts.kClimbSpeed);
        } else if (rightButtonDown) {
            climber.climbRight(ClimberConsts.kClimbSpeed);
        } else if (leftButtonDown) {
            climber.climbLeft(ClimberConsts.kClimbSpeed);
        } else {
            climber.climberStop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
