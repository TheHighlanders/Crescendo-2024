package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

        /*----------------------------------------------------------------------------*/
        /* Bottom */
        /*----------------------------------------------------------------------------*/
        bottomFlywheelMotor =
            new CANSparkMaxCurrent(Constants.Shooter.bottomFlywheelMotorID, MotorType.kBrushless);
        bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
        bottomFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kBottomRatio);
        bottomFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pidBottom = bottomFlywheelMotor.getPIDController();
        pidBottom.setOutputRange(
            Constants.Shooter.PIDValues.minOut,
            Constants.Shooter.PIDValues.maxOut
        );
        pidBottom.setP(Constants.Shooter.PIDValues.kP);
        pidBottom.setI(Constants.Shooter.PIDValues.kI);
        pidBottom.setD(Constants.Shooter.PIDValues.kD);
        pidBottom.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
        /*----------------------------------------------------------------------------*/
        /* Top */
        /*----------------------------------------------------------------------------*/
        topFlywheelMotor =
            new CANSparkMaxCurrent(Constants.Shooter.topFlywheelMotorID, MotorType.kBrushless);
        topFlywheelEncoder = topFlywheelMotor.getEncoder();
        topFlywheelEncoder.setPositionConversionFactor(Constants.Shooter.kTopRatio);
        topFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pidTop = topFlywheelMotor.getPIDController();
        pidTop.setOutputRange(
            Constants.Shooter.PIDValues.minOut,
            Constants.Shooter.PIDValues.maxOut
        );
        pidTop.setP(Constants.Shooter.PIDValues.kP);
        pidTop.setI(Constants.Shooter.PIDValues.kI);
        pidTop.setD(Constants.Shooter.PIDValues.kD);
        pidTop.setIMaxAccum(Constants.Shooter.PIDValues.iMaxAccum, Constants.Shooter.slotID);
    }

    public void shoot() {
        pidBottom.setReference(1, CANSparkMax.ControlType.kVelocity);
        pidTop.setReference(1, CANSparkMax.ControlType.kVelocity);
    }

    public void shootCancel() {
        pidBottom.setReference(0, CANSparkMax.ControlType.kVelocity);
        pidTop.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    public boolean hasGamePiece() {
        // TODO: tell when the game peace leaves the shooter (could be button press or
        // check current draw)
        return true;
    }

    @Override
    public void periodic() {}
}
