// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEst0;
    PhotonPoseEstimator photonPoseEst1;
    PhotonCamera cam0;
    PhotonCamera cam1;

    /** Creates a new Vision. */
    public Vision() {
        cam0 = new PhotonCamera("Arducam_OV9281_USB_Camera");
        cam1 = new PhotonCamera("6201_Cam_1");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            DriverStation.reportWarning("AprilTag Field Layout Load Exception", true);
        }

        photonPoseEst0 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam0, VisionConstants.kRobotCamera0);

        photonPoseEst1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1, VisionConstants.kRobotCamera1);
    }

    @Override
    public void periodic() {}

    public Optional<EstimatedRobotPose> getEstimatedGlobalPosePhoton0(Pose2d prevEstimatedRobotPose) {
        photonPoseEst0.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEst0.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPosePhoton1(Pose2d prevEstimatedRobotPose) {
        photonPoseEst1.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEst1.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPosePhoton0() {
        return photonPoseEst0.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPosePhoton1() {
        return photonPoseEst1.update();
    }

    public PhotonPipelineResult getLatestResult0() {
        return cam0.getLatestResult();
    }

    public PhotonPipelineResult getLatestResult1() {
        return cam1.getLatestResult();
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, int camID) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = ((camID == 0) ? getLatestResult0().getTargets() : getLatestResult1().getTargets());
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonPoseEst0.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); else estStdDevs =
            estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
