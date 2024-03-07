// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConsts;
import frc.robot.util.CANSparkMaxCurrent;

public class Climber extends SubsystemBase {

    public CANSparkMaxCurrent climberMotorRight;
    public CANSparkMaxCurrent climberMotorLeft;

    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;

    public SparkPIDController leftPID;
    public SparkPIDController rightPID;

    /** Creates a new intake. */
    public Climber() {
        climberMotorLeft = new CANSparkMaxCurrent(Constants.ClimberConsts.CLIMBER_LEFT, MotorType.kBrushless);
        climberMotorLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);

        climberMotorRight = new CANSparkMaxCurrent(Constants.ClimberConsts.CLIMBER_RIGHT, MotorType.kBrushless);
        climberMotorRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        
        leftEncoder = climberMotorLeft.getEncoder();
        leftEncoder.setPosition(0);
        rightEncoder = climberMotorRight.getEncoder();
        rightEncoder.setPosition(0);

        leftPID = climberMotorLeft.getPIDController();
        rightPID = climberMotorRight.getPIDController();

        leftPID.setP(ClimberConsts.kClimberP);
        leftPID.setOutputRange(0, 0.25);

        rightPID.setP(ClimberConsts.kClimberP);
        rightPID.setOutputRange(0, 0.25);

    }

    @Override
    public void periodic() {}

    public void climbBoth(double speed) {
        climberMotorRight.set(speed);
        climberMotorLeft.set(-speed);
    }

    public void climbLeft(double speed) {
        climberMotorLeft.set(-speed);
    }

    public void climbRight(double speed) {
        climberMotorRight.set(speed);
    }

    public void climberStop() {
        climberMotorRight.set(0);
        climberMotorLeft.set(0);
    }

    public void climberPrime(){
        leftPID.setReference(ClimberConsts.kClimberPrimePoint, ControlType.kPosition);
        rightPID.setReference(ClimberConsts.kClimberPrimePoint, ControlType.kPosition);
    }
}
