// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConsts;
import frc.robot.util.CANSparkMaxCurrent;

public class Climber extends SubsystemBase {

    public CANSparkMaxCurrent climberMotorRight;
    public CANSparkMaxCurrent climberMotorLeft;

    public Servo servoLeft;
    public Servo servoRight;

    public boolean servoLeftOut = false;
    public boolean servoRightOut = false;

    /** Creates a new intake. */
    public Climber() {
        climberMotorLeft = new CANSparkMaxCurrent(Constants.ClimberConsts.CLIMBER_LEFT, MotorType.kBrushless);
        climberMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

        climberMotorRight = new CANSparkMaxCurrent(Constants.ClimberConsts.CLIMBER_RIGHT, MotorType.kBrushless);
        climberMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        servoLeft = new Servo(ClimberConsts.kServoRightID);
        servoRight = new Servo(ClimberConsts.kServoLeftID);

        servoLeft.set(0);
        servoRight.set(0);
    }

    @Override
    public void periodic() {}

    public void climbBoth(double speed) {
        climberMotorRight.set(speed);
        climberMotorLeft.set(-speed);

        if (!servoLeftOut) {
            servoLeft.set(1);
            servoLeftOut = true;
        }

        if (!servoRightOut) {
            servoRight.set(1);
            servoRightOut = true;
        }
    }

    public void climbLeft(double speed) {
        climberMotorLeft.set(-speed);

        if (!servoLeftOut) {
            servoLeft.set(1);
            servoLeftOut = true;
        }
    }

    public void climbRight(double speed) {
        climberMotorRight.set(speed);

        if (!servoRightOut) {
            servoRight.set(1);
            servoRightOut = true;
        }
    }

    public void climberStop() {
        climberMotorRight.set(0);
        climberMotorLeft.set(0);
    }
}
