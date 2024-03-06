package frc.robot.subsystems;

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

    public Shooter() {
        beamBreak = new DigitalInput(Constants.Shooter.kShooterBeamBreakDIOPin);
        /*----------------------------------------------------------------------------*/
        /* Bottom */
        /*----------------------------------------------------------------------------*/
        bottomFlywheelMotor = new CANSparkMaxCurrent(Constants.Shooter.bottomFlywheelMotorID, MotorType.kBrushless);
        bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
        bottomFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kBottomGearRatio);
        //bottomFlywheelEncoder.setVelocityConversionFactor(Constants.Shooter.kBottomVelocityConversionFactor);
        bottomFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pidBottom = bottomFlywheelMotor.getPIDController();
        pidBottom.setOutputRange(Constants.Shooter.PIDValues.minOut, Constants.Shooter.PIDValues.maxOut);
        pidBottom.setP(Constants.Shooter.PIDValues.kP);
        pidBottom.setI(Constants.Shooter.PIDValues.kI);
        pidBottom.setD(Constants.Shooter.PIDValues.kD);
        pidBottom.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
        /*----------------------------------------------------------------------------*/
        /* Top */
        /*----------------------------------------------------------------------------*/
        topFlywheelMotor = new CANSparkMaxCurrent(Constants.Shooter.topFlywheelMotorID, MotorType.kBrushless);
        topFlywheelEncoder = topFlywheelMotor.getEncoder();
        topFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kTopRatio);
        topFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pidTop = topFlywheelMotor.getPIDController();
        pidTop.setOutputRange(Constants.Shooter.PIDValues.minOut, Constants.Shooter.PIDValues.maxOut);
        pidTop.setP(Constants.Shooter.PIDValues.kP);
        pidTop.setI(Constants.Shooter.PIDValues.kI);
        pidTop.setD(Constants.Shooter.PIDValues.kD);
        pidTop.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
    }

    public void shoot(DoubleSupplier speed) {
        pidBottom.setReference(speed.getAsDouble(), CANSparkMax.ControlType.kVelocity);
        pidTop.setReference(speed.getAsDouble(), CANSparkMax.ControlType.kVelocity);
        // topFlywheelMotor.set(speed.getAsDouble());
        // bottomFlywheelMotor.set(speed.getAsDouble());
    }

    public void shootCancel() {
        pidBottom.setReference(0, CANSparkMax.ControlType.kVelocity);
        pidTop.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Shooter Speed", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Shooter Speed", bottomFlywheelEncoder.getVelocity());
    }
}
