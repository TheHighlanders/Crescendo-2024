// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Modules;
import frc.robot.RobotContainer;
import java.util.Arrays;

public class Swerve extends SubsystemBase {

    /* Array of Modules */
    public SwerveModule[] modules;
    private AHRS gyro;
    public ChassisSpeeds chassisSpeeds;

    public SwerveDriveOdometry odometer;

    public Swerve() {
        /* Initializes modules from Constants */
        modules =
            new SwerveModule[] {
                new SwerveModule(0, Modules.FrontLeft.FL0),
                new SwerveModule(1, Modules.FrontRight.FR1),
                new SwerveModule(2, Modules.BackLeft.BL2),
                new SwerveModule(3, Modules.BackRight.BR3),
            };

        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();

        // SmartDashboard.putNumber("getRobotRelativeSpeedsX", 1);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsY", 2);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsO", 3);

        // SmartDashboard.putNumber("PP X", 4);
        // SmartDashboard.putNumber("PP Y", 5);
        // SmartDashboard.putNumber("PP O", 6);

        odometer = new SwerveDriveOdometry(Constants.SwerveConst.kinematics, gyro.getRotation2d(), getModulePositions());

        chassisSpeeds = new ChassisSpeeds();
        // SmartDashboard.putData(field);

        /* <pp copied> */
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Autonomous.pathFollowConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
        /* <pp copied\> */
    }

    @Override
    public void periodic() {
        odometer.update(gyro.getRotation2d(), getModulePositions());
        // SmartDashboard.putNumber("ODOX2", odometer.getPoseMeters().getX());

        // SmartDashboard.putNumber("xCalc", -1);
        // SmartDashboard.putNumber("yCalc", -1);
        // SmartDashboard.putBoolean("Running", false);
        sendSmartDashboardDiagnostics();
        // field.setRobotPose(getPose());
        for (SwerveModule m : modules) {
            m.runPeriodicLimiting();
        }
    }

    /**
     * Runs all IK and sets modules states
     *
     * @param translate     Desired translations speeds m/s
     * @param rotate        Desired rotation rate Rotation2d
     * @param fieldRelative Driving mode
     * @param isOpenLoop    Drive controller mode
     */
    public void drive(Translation2d translate, Rotation2d rotate, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds =
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-translate.getX(), -translate.getY(), rotate.getRadians(), getYaw())
                : new ChassisSpeeds(translate.getX(), translate.getY(), rotate.getRadians());

        SwerveModuleState[] swerveModuleStates = Constants.SwerveConst.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConst.kMaxSpeedTele);

        for (SwerveModule m : modules) {
            m.setModuleState(swerveModuleStates[m.moduleNumber], isOpenLoop); //WHY WHY WHY
        }
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        // SmartDashboard.putNumber("PP X", chassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("PP Y", chassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("PP O", chassisSpeeds.omegaRadiansPerSecond);

        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.SwerveConst.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConst.kMaxSpeedTele);

        for (SwerveModule m : modules) {
            m.setModuleState(swerveModuleStates[m.moduleNumber], Constants.SwerveConst.kOpenLoop);
        }
    }

    /**
     * Zeros the NavX
     */
    public void zeroGyro() {
        gyro.zeroYaw();
    }

    // For PP
    public void resetPose(Pose2d pose) {
        RobotContainer.s_Localizer.resetOdoPose2d(pose);
        odometer.resetPosition(getYaw(), getModulePositions(), pose);

        // SmartDashboard.putNumber("ResetPoseX", pose.getX());
        // SmartDashboard.putNumber("ResetPoseY", pose.getY());
    }

    /**
     * Returns the gyro's yaw
     *
     * @return Yaw of gyro, includes zeroing
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-1 * gyro.getYaw());
    }

    /**
     * Gets swerve modules positions for all modules
     *
     * @return Array of modules positions, in modules ID order
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // return new Pose2d(odometer.getPoseMeters().getTranslation(), new Rotation2d());
        // return odometer.getPoseMeters();
        // return swervePoseEstimator.getEstimatedPosition();
        return RobotContainer.getLocalizedPose.get();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // SmartDashboard.putNumber("getRobotRelativeSpeedsX", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).vxMetersPerSecond);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsY", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).vyMetersPerSecond);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsO", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond);

        return Constants.SwerveConst.kinematics.toChassisSpeeds(getStates());
    }

    /**
     * Gets swerve modules states for all modules
     *
     * @return Array of modules states, in modules ID order
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void resetAllModulestoAbsol() {
        for (SwerveModule m : modules) {
            m.setIntegratedAngleToAbsolute();
        }
    }

    /**
     *  Sends actual angle encoder data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendAngleTargetDiagnostic()
     */
    public void sendAngleDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Angle Actual", m.angleEncoder.getPosition());
        }
    }

    /**
     *  Sends angle PID target data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendAngleDiagnostic()
     */
    public void sendAngleTargetDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Angle Target", m.angleReference);
        }
    }

    /**
     *  Sends actual drive encoder data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendDriveTargetDiagnostic()
     */
    public void sendDriveDiagnostic() {
        double[] wheelSpeeds = new double[4];
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Velocity Actual", m.driveEncoder.getVelocity());
            wheelSpeeds[m.driveMotor.getDeviceId() / 10] = m.driveEncoder.getVelocity();
        }

        SmartDashboard.putNumber(
            "Velocity Range",
            Math.abs(Arrays.stream(wheelSpeeds).max().getAsDouble()) - Math.abs(Arrays.stream(wheelSpeeds).min().getAsDouble())
        );
    }

    /**
     *  Sends drive PID target data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendDriveDiagnostic()
     */
    public void sendDriveTargetDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Velocity Target", m.driveReference);
        }
    }

    public void sendAbsoluteDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Absolute", m.getAbsolutePositionNoOffset().getDegrees());
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Absolute Offsetted", m.getAbsolutePosition().getDegrees());
        }
    }

    public void sendSmartDashboardDiagnostics() {
        sendAngleDiagnostic();
        sendAngleTargetDiagnostic();

        // sendDriveDiagnostic();
        // sendDriveTargetDiagnostic();

        sendAbsoluteDiagnostic();

        SmartDashboard.putNumber("NavX Angle", getYaw().getDegrees());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Theta", getPose().getRotation().getDegrees());
    }

    public void jogSingleModule(int moduleNumber, double input, boolean drive) {
        if (drive) {
            modules[moduleNumber].setDriveState(new SwerveModuleState(input, new Rotation2d(0)), false);
            DriverStation.reportWarning(modules[moduleNumber].driveMotor.getDeviceId() + "", false);
        } else {
            modules[moduleNumber].setAngleState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(input))));
            DriverStation.reportWarning(modules[moduleNumber].angleMotor.getDeviceId() + "", false);
        }
    }

    public void jogAllModuleDrive(double v) {
        drive(new Translation2d(0, v), new Rotation2d(0), true, false);
    }
}
