package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {

  public int driveMotorID;
  public int angleMotorID;

  public Rotation2d absoluteEncoderOffset;

  public SwerveModuleConfig(
    int driveMotorID,
    int angleMotorID,
    Rotation2d absoluteEncoderOffset
  ) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;

    this.absoluteEncoderOffset = absoluteEncoderOffset;
  }
}
