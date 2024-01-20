// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Localizer extends SubsystemBase {
    SwerveDrivePoseEstimator swervePoseEstimator;
    Swerve swerve;
    Vision vision;
  /** Creates a new Localizer. */
  public Localizer(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
    
    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConst.kinematics, this.swerve.getYaw(), this.swerve.getModulePositions(),
      new Pose2d());


  }

  @Override
  public void periodic() {
    swervePoseEstimator.update(this.swerve.getYaw(), this.swerve.getModulePositions());
    
    Optional<EstimatedRobotPose> estPose = vision.getEstimatedGlobalPosePhoton(null);
    
    if(estPose.isPresent()){
      swervePoseEstimator.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(),estPose.get().timestampSeconds);
    }
    // This method will be called once per scheduler run
  }
    
  public void resetOdoPose2d(Pose2d pose){
    swervePoseEstimator.resetPosition(this.swerve.getYaw(), this.swerve.getModulePositions(), pose);
  }

  public Pose2d getPose(){
    return swervePoseEstimator.getEstimatedPosition();
  }
}
