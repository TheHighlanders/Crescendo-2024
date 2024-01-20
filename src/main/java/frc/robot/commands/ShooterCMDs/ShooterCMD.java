package frc.robot.commands.ShooterCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCMD extends Command {

  private final Shooter m_shooter;

  public ShooterCMD(Shooter snoot) {
    m_shooter = snoot;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.shoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
