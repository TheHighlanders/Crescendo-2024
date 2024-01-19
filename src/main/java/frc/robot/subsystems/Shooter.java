import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCMD;
import java.util.function.BooleanSupplier;

public class Shooter extends SubsystemBase {

  

  public Shooter() {}

  private BooleanSupplier m_hasGamePiece = () -> false;
  private BooleanSupplier m_pivotAligned = () -> false;

  
  public void setSuppliers(BooleanSupplier hasGamePiece, BooleanSupplier pivotAligned) {
    m_hasGamePiece = hasGamePiece;
    m_pivotAligned = pivotAligned;
  }

  // /**
  //  * command factory
  //  *
  //  * @return a command
  //  */

  // public Command Eject() {}

  public boolean shoot() {
    return false;
  }

  public boolean shoot() {
    return false;
  }

  @Override
  public void periodic() {}
}
