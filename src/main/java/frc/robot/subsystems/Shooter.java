package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxCurrent;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {

    private CANSparkMaxCurrent bottomFlywheelMotor;
    private CANSparkMaxCurrent topFlywheelMotor;
    private RelativeEncoder bottomFlywheelEncoder;
    private RelativeEncoder topFlywheelEncoder;

    private SparkPIDController pidBottom;
    private SparkPIDController pidTop;

    private DigitalInput beamBreak;

    private double setpoint;

    public Shooter() {
        beamBreak = new DigitalInput(Constants.Shooter.kShooterBeamBreakDIOPin);
        setpoint = 0;
        /*----------------------------------------------------------------------------*/
        /* Bottom */
        /*----------------------------------------------------------------------------*/
        bottomFlywheelMotor = new CANSparkMaxCurrent(Constants.Shooter.bottomFlywheelMotorID, MotorType.kBrushless);
        bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
        bottomFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kBottomGearRatio);
        // bottomFlywheelMotor.setSpikeCurrentLimit(Constants.Shooter.ShooterCurrentLimit.kLimitToAmps, Constants.Shooter.ShooterCurrentLimit.kMaxSpikeTime, Constants.Shooter.ShooterCurrentLimit.kMaxSpikeAmps, Constants.Shooter.ShooterCurrentLimit.kSmartLimit);
        bottomFlywheelMotor.setSmartCurrentLimit(frc.robot.Constants.Shooter.kCurrentLimit);
        bottomFlywheelEncoder.setVelocityConversionFactor(Constants.Shooter.kBottomVelocityConversionFactor);
        bottomFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pidBottom = bottomFlywheelMotor.getPIDController();
        pidBottom.setOutputRange(Constants.Shooter.PIDValues.minOut, Constants.Shooter.PIDValues.maxOut);
        pidBottom.setP(Constants.Shooter.PIDValues.kP);
        pidBottom.setI(Constants.Shooter.PIDValues.kI);
        pidBottom.setD(Constants.Shooter.PIDValues.kD);
        pidBottom.setFF(0.6 / 3200d);
        pidBottom.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
        /*----------------------------------------------------------------------------*/
        /* Top */
        /*----------------------------------------------------------------------------*/
        topFlywheelMotor = new CANSparkMaxCurrent(Constants.Shooter.topFlywheelMotorID, MotorType.kBrushless);
        //topFlywheelMotor.setSpikeCurrentLimit(Constants.Shooter.ShooterCurrentLimit.kLimitToAmps, Constants.Shooter.ShooterCurrentLimit.kMaxSpikeTime, Constants.Shooter.ShooterCurrentLimit.kMaxSpikeAmps, Constants.Shooter.ShooterCurrentLimit.kSmartLimit);
        topFlywheelMotor.setSmartCurrentLimit(frc.robot.Constants.Shooter.kCurrentLimit);
        topFlywheelEncoder = topFlywheelMotor.getEncoder();
        topFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kTopRatio);
        topFlywheelEncoder.setVelocityConversionFactor(Constants.Shooter.kBottomVelocityConversionFactor);
        topFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pidTop = topFlywheelMotor.getPIDController();
        pidTop.setOutputRange(Constants.Shooter.PIDValues.minOut, Constants.Shooter.PIDValues.maxOut);
        pidTop.setP(Constants.Shooter.PIDValues.kP);
        pidTop.setI(Constants.Shooter.PIDValues.kI);
        pidTop.setD(Constants.Shooter.PIDValues.kD);
        pidTop.setFF(0.6 / 3200d);
        pidTop.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
    }

    public void shoot(DoubleSupplier speed) {
        setpoint = speed.getAsDouble();
        pidBottom.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        pidTop.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    }

    public void shoot(double speed) {
        pidBottom.setReference(speed, CANSparkMax.ControlType.kVelocity);
        pidTop.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public boolean atVelocity() {
        return (
            Math.abs(bottomFlywheelEncoder.getVelocity() - setpoint) < Constants.Shooter.velocityTolerance &&
            Math.abs(topFlywheelEncoder.getVelocity() - setpoint) < Constants.Shooter.velocityTolerance
        );
    }

    public boolean aboveMinVelocity() {
        return bottomFlywheelEncoder.getVelocity() > Constants.Shooter.velocityMinimum;
    }

    public void shootCancel() {
        pidBottom.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        pidTop.setReference(0, CANSparkMax.ControlType.kDutyCycle);
    }

    public void shootIdle() {
        pidBottom.setReference(3000, ControlType.kVelocity);
        pidTop.setReference(3000, ControlType.kVelocity);
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("at shooter velocity setpoint", atVelocity());

        topFlywheelMotor.periodicLimit();
        bottomFlywheelMotor.periodicLimit();
        SmartDashboard.putNumber("!Shooter velocity", topFlywheelEncoder.getVelocity());
    }
}
