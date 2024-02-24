package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxCurrent;

public class Shooter extends SubsystemBase {

    private CANSparkMaxCurrent bottomFlywheelMotor;
    private CANSparkMaxCurrent topFlywheelMotor;
    private RelativeEncoder bottomFlywheelEncoder;
    private RelativeEncoder topFlywheelEncoder;

    private SparkPIDController pidBottom;
    private SparkPIDController pidTop;

    public Shooter() {
        bottomFlywheelMotor = new CANSparkMaxCurrent(Constants.Shooter.bottomFlywheelMotorID,
                MotorType.kBrushless);
        bottomFlywheelMotor.setCurrent(1);
        bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
        bottomFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kBottomGearRatio);
        bottomFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);



        pidBottom = bottomFlywheelMotor.getPIDController();
        // pidBottom.setOutputRange(Constants.Shooter.PIDValues.minOut,
        //         Constants.Shooter.PIDValues.maxOut);
        pidBottom.setP(Constants.Shooter.PIDValues.kP);
        pidBottom.setI(Constants.Shooter.PIDValues.kI);
        pidBottom.setD(Constants.Shooter.PIDValues.kD);
        // pidBottom.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
        // pidBottom.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, Constants.Shooter.slotID);
        // pidBottom.setSmartMotionMinOutputVelocity(Constants.Shooter.minVel,
        //         Constants.Shooter.slotID);
        // pidBottom.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, Constants.Shooter.slotID);
        // pidBottom.setSmartMotionAllowedClosedLoopError(Constants.Shooter.allowedErr,
        //         Constants.Shooter.slotID);

        topFlywheelMotor =
                new CANSparkMaxCurrent(Constants.Shooter.topFlywheelMotorID, MotorType.kBrushless);
        topFlywheelMotor.setCurrent(1);
        topFlywheelEncoder = topFlywheelMotor.getEncoder();
        topFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kTopRatio);
        topFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pidTop = topFlywheelMotor.getPIDController();
        // pidTop.setOutputRange(Constants.Shooter.PIDValues.minOut,
        //         Constants.Shooter.PIDValues.maxOut);
        pidTop.setP(Constants.Shooter.PIDValues.kP);
        pidTop.setI(Constants.Shooter.PIDValues.kI);
        pidTop.setD(Constants.Shooter.PIDValues.kD);
        // pidTop.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
        // pidTop.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, Constants.Shooter.slotID);
        // pidTop.setSmartMotionMinOutputVelocity(Constants.Shooter.minVel, Constants.Shooter.slotID);
        // pidTop.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, Constants.Shooter.slotID);
        // pidTop.setSmartMotionAllowedClosedLoopError(Constants.Shooter.allowedErr,
        //         Constants.Shooter.slotID);
    }

    public void shoot() {
        pidTop.setReference(1, CANSparkMax.ControlType.kSmartVelocity);
        pidBottom.setReference(1, CANSparkMax.ControlType.kSmartVelocity);
        DriverStation.reportWarning("Shooting", false);
    }

    public void shootCancel() {
        topFlywheelMotor.set(0);
        bottomFlywheelMotor.set(0);
    }

    public boolean hasGamePiece() {
        // TODO: tell when the game peace leaves the shooter (could be button press or
        // check current draw)
        return true;
    }

    @Override
    public void periodic() {}
}
