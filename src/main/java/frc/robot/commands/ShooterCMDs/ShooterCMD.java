package frc.robot.commands.ShooterCMDs;

import frc.robot.subsystems.Shooter;

public class ShooterCMD extends Command {

  private final Shooter m_shooter;

  public ShooterCMD(Shooter snoot) {
    m_shooter = snoot;
    addRequirements(m_shooter);
  }

  @Override
  protected void initialize() {}

  @Override
  protected void execute() {}

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {}

  @Override
  protected void interrupted() {}
}
