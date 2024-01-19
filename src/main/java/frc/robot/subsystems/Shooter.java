import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCMD;
import java.util.function.BooleanSupplier;

public class Shooter extends SubsystemBase {

  private BotPoseProvider m_source;
  private BooleanSupplier m_hasGamePiece = () -> false;
  private BooleanSupplier m_pivotAligned = () -> false;

  public Shooter() {}

  public void setSuppliers(
    BooleanSupplier hasGamePiece,
    BooleanSupplier pivotAligned
  ) {
    m_hasGamePiece = hasGamePiece;
    m_pivotAligned = pivotAligned;
  }

  public void shoot() {
    if (!m_pivotAligned) {
      //call alignPivot(calcTrajectory());
    }
    if (!m_hasGamePiece) {
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
