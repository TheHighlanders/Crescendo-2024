package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxCurrent;
import java.util.function.BooleanSupplier;

public class Shooter extends SubsystemBase {

  public CANSparkMaxCurrent bottomFlywheelMotor;
  public CANSparkMaxCurrent topFlywheelMotor;
  public RelativeEncoder bottomFlywheelEncoder;
  public RelativeEncoder topFlywheelEncoder;

  public SparkPIDController pidBottom;
  public SparkPIDController pidTop;

  private BooleanSupplier m_hasGamePiece = () -> false;
  private BooleanSupplier m_pivotAligned = () -> false;

  public Shooter() {
    bottomFlywheelMotor =
      new CANSparkMaxCurrent(
        Constants.Shooter.bottomFlywheelMotorID,
        MotorType.kBrushless
      );
    bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
    bottomFlywheelEncoder.setPositionConversionFactor(
      Constants.Shooter.kBottomRatio
    );
    bottomFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    pidBottom = bottomFlywheelMotor.getPIDController();
    pidBottom.setOutputRange(
      Constants.Shooter.pidValues.minOut,
      Constants.Shooter.pidValues.maxOut
    );
    pidBottom.setP(Constants.Shooter.pidValues.kP);
    pidBottom.setI(Constants.Shooter.pidValues.kI);
    pidBottom.setD(Constants.Shooter.pidValues.kD);
    pidBottom.setIMaxAccum(
      Constants.Shooter.pidValues.iMaxAccum,
      Constants.Shooter.slotID
    );
    pidBottom.setSmartMotionMaxVelocity(Constants.Shooter.maxVel, Constants.Shooter.slotID);
    pidBottom.setSmartMotionMinOutputVelocity(Constants.Shooter.minVel, Constants.Shooter.slotID);
    pidBottom.setSmartMotionMaxAccel(Constants.Shooter.maxAcc, Constants.Shooter.slotID);
    pidBottom.setSmartMotionAllowedClosedLoopError(Constants.Shooter.allowedErr, Constants.Shooter.slotID);
  }

  public void setSuppliers(
    BooleanSupplier hasGamePiece,
    BooleanSupplier pivotAligned
  ) {
    m_hasGamePiece = hasGamePiece;
    m_pivotAligned = pivotAligned;
  }

  public void shoot() {
    if (!m_pivotAligned.getAsBoolean()) {
      //call alignPivot(calcTrajectory());
    }
    if (!m_hasGamePiece.getAsBoolean()) {
      // well that sucks
    }
    // tell pivot to align shot
  }

  public double calcTrajectory() {
    return 1_000;
  }

  @Override
  public void periodic() {}
}
