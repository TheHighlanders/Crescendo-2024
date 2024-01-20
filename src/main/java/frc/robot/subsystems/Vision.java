// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEst0;
  PhotonCamera cam0;


  /** Creates a new Vision. */
  public Vision() {
    cam0 = new PhotonCamera("6201Cam0");

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e){
      e.printStackTrace();
      DriverStation.reportWarning("AprilTag Field Layout Load Exception", true);
    }

    photonPoseEst0 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam0,  Constants.Autonomous.kRobotCamera0);
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Optional<EstimatedRobotPose> getEstimatedGlobalPosePhoton(Pose2d prevEstimatedRobotPose) {
    photonPoseEst0.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEst0.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPosePhoton() {
    return photonPoseEst0.update();
  }
}
