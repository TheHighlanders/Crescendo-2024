package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxCurrent;

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
      Constants.Shooter.pidConstants.minOut,
      Constants.Shooter.pidConstants.maxOut
    );
    pidBottom.setP(Constants.Shooter.pidConstants.kP);
    pidBottom.setI(Constants.Shooter.pidConstants.kI);
    pidBottom.setD(Constants.Shooter.pidConstants.kD);
    pidBottom.setIMaxAccum(
      Constants.Shooter.pidConstants.iMaxAccum,
      Constants.Shooter.pidConstants.slotID
    );

    

    topFlywheelMotor =
      new CANSparkMaxCurrent(
        Constants.Shooter.topFlywheelMotorID,
        MotorType.kBrushless
      );
    topFlywheelEncoder = topFlywheelMotor.getEncoder();
    topFlywheelEncoder.setPositionConversionFactor(
      Constants.Shooter.kTopRatio
    );
    topFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    pidTop = topFlywheelMotor.getPIDController();
    pidTop.setOutputRange(
      Constants.Shooter.pidConstants.minOut,
      Constants.Shooter.pidConstants.maxOut
    );
    pidTop.setP(Constants.Shooter.pidConstants.kP);
    pidTop.setI(Constants.Shooter.pidConstants.kI);
    pidTop.setD(Constants.Shooter.pidConstants.kD);
    pidTop.setIMaxAccum(
      Constants.Shooter.pidConstants.iMaxAccum,
      Constants.Shooter.pidConstants.slotID
    );
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
    // tell drive to align shot
  }

  public double calcTrajectory() {
    return 1_000;
  }

  @Override
  public void periodic() {}
}
