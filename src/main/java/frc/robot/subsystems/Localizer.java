// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Localizer extends SubsystemBase {

    SwerveDrivePoseEstimator swervePoseEstimator;
    Pose2d previous0 = new Pose2d();
    Pose2d previous1 = new Pose2d();
    Field2d field;
    Swerve swerve;
    Vision vision;

    /** Creates a new Localizer. */
    public Localizer(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

        field = new Field2d();

        swervePoseEstimator =
            new SwerveDrivePoseEstimator(
                Constants.SwerveConst.kinematics,
                this.swerve.getYaw(),
                this.swerve.getModulePositions(),
                new Pose2d()
            );

        SmartDashboard.putData(field);
    }

    @Override
    public void periodic() {
        swervePoseEstimator.update(this.swerve.getYaw(), this.swerve.getModulePositions());

        Optional<EstimatedRobotPose> estPose0;
        Optional<EstimatedRobotPose> estPose1;


        if (previous0 != null) {
            estPose0 = vision.getEstimatedGlobalPosePhoton0(previous0);

        } else {
            estPose0 = vision.getEstimatedGlobalPosePhoton0();
        }

        if (previous1 != null) {
            estPose1 = vision.getEstimatedGlobalPosePhoton1(previous0);

        } else {
            estPose1 = vision.getEstimatedGlobalPosePhoton1();
        }

        if (estPose0.isPresent()) {
            var estStdDevs = vision.getEstimationStdDevs(estPose0.get().estimatedPose.toPose2d(), 0);
            previous0 = estPose0.get().estimatedPose.toPose2d();
            swervePoseEstimator.addVisionMeasurement(
                estPose0.get().estimatedPose.toPose2d(),
                estPose0.get().timestampSeconds,
                estStdDevs
            );
        }

        if (estPose1.isPresent()) {
            var estStdDevs = vision.getEstimationStdDevs(estPose1.get().estimatedPose.toPose2d(), 1);
            previous1 = estPose1.get().estimatedPose.toPose2d();
            swervePoseEstimator.addVisionMeasurement(
                estPose1.get().estimatedPose.toPose2d(),
                estPose1.get().timestampSeconds,
                estStdDevs
            );
        }

        field.setRobotPose(getPose());
        // field.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        // field.setRobotPose(getPose().getX(), getPose().getY(),
        // getPose().getRotation());
    }

    public void resetOdoPose2d(Pose2d pose) {
        swervePoseEstimator.resetPosition(
            this.swerve.getYaw(),
            this.swerve.getModulePositions(),
            pose
        );
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public Field2d getField() {
        return field;
    }

    public double getDistanceToSpeaker() {
        Translation2d robot = getPose().getTranslation();
        Translation2d goal =
            (
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? Constants.VisionConstants.kRedSpeaker
                    : Constants.VisionConstants.kBlueSpeaker
            );

        return Math.hypot(robot.getX() - goal.getX(), robot.getY() - goal.getY());
    }

    public Rotation2d getAngleToSpeaker() {
        Translation2d robot = getPose().getTranslation();
        Translation2d goal =
            (
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? Constants.VisionConstants.kRedSpeaker
                    : Constants.VisionConstants.kBlueSpeaker
            );
        SmartDashboard.putNumber("Angle to Speaker", Math.toDegrees(Math.atan2(goal.getY()-robot.getY(), goal.getX() - robot.getX())));
        return new Rotation2d(Math.atan2(goal.getY()-robot.getY(), goal.getX() - robot.getX()));
    }
}
