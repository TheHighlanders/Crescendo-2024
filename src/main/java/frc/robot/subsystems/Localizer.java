// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Autonomous;

public class Localizer extends SubsystemBase {
    SwerveDrivePoseEstimator swervePoseEstimator;
    Pose2d previous = new Pose2d();
    Field2d field;
    Swerve swerve;
    Vision vision;
  /** Creates a new Localizer. */
  public Localizer(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;

    field = new Field2d();
    
    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConst.kinematics, this.swerve.getYaw(), this.swerve.getModulePositions(),
      new Pose2d());

    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    swervePoseEstimator.update(this.swerve.getYaw(), this.swerve.getModulePositions());
    // swervePoseEstimator.update(this.swerve.getYaw(), new SwerveModulePosition[]  {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()});
    
    Optional<EstimatedRobotPose> estPose;

    if(previous != null){
      estPose = vision.getEstimatedGlobalPosePhoton(previous);
    } else {
      estPose = vision.getEstimatedGlobalPosePhoton();
    }


    if(estPose.isPresent()){
      previous = estPose.get().estimatedPose.toPose2d();
      swervePoseEstimator.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);
    }


    field.setRobotPose(getPose());
  }
    
  public void resetOdoPose2d(Pose2d pose){
    swervePoseEstimator.resetPosition(this.swerve.getYaw(), this.swerve.getModulePositions(), pose);
  }

  public Pose2d getPose(){
    return swervePoseEstimator.getEstimatedPosition();
  }

  public Field2d getField(){
    return field;
  }

  public double getDistanceToSpeaker(){
    Translation2d robot = getPose().getTranslation();
    Translation2d goal = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Autonomous.kRedSpeaker : Autonomous.kBlueSpeaker);

    return Math.hypot(robot.getX() - goal.getX(), robot.getY() - goal.getY());
  }
}