// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class SwerveConst {

        public static final boolean kOpenLoop = true;

        public static final double kTranslateP = 5.0;
        public static final double kTranslateI = 0.1;
        public static final double kTranslateD = 0.13;

        public static final double kRotateP = 5.0;
        public static final double kRotateI = 0.0;
        public static final double kRotateD = 0.13;

        public static final double kMaxSpeedTele = 3.0; //Meters per Second
        public static final double kMaxAngularSpeedFast = Math.PI; //Degrees per Second

        public static final double kStickDeadband = 0.01;

        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        public static final double kWheelBase = Units.inchesToMeters(23.5);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        );

        public static double speedLimit = 3.0;
        public static double slowSpeedLimit = 1.0;

        public static double accelerationLimit = 1.5;
        public static double slowAccelerationLimit = 2;

        public static double angularVelocityLimit = 180.0;
        public static double slowAngularVelocityLimit = 45.0;
    }

    public static class Autonomous {

        public static final HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1.1, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.35, 0.00, 0.00), // Rotation PID constants //0.004 0.01 0.01
            2, // Max module speed, in m/s //used to be 1, changed to rotate faster AND IT WORKS!
            0.42207203769, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
    }

    public static class VisionConstants {

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1, 1, 2); // VecBuilder.fill(0.00040, 0.00080, 0.00050);
        //TODO: Actual stddevs

        public static final Translation2d kBlueSpeaker = new Translation2d(-0.04, 5.55);
        public static final Translation2d kRedSpeaker = new Translation2d(16.58, 5.55);

        public static final Translation3d robotCameraTranslation0 = new Translation3d(-0.0445, -0.3175, 0.1778); //-x, -y, z
        public static final Rotation3d robotCameraRotation0 = new Rotation3d(0, 0, -Math.PI / 2.0f);
        public static final Transform3d kRobotCamera0 = new Transform3d(robotCameraTranslation0, robotCameraRotation0);
    }

    public static class Module {

        public static class DriveCurrentLimit {

            public static final double kLimitToAmps = 40.0f;
            public static final double kMaxSpikeTime = 25.0f;
            public static final double kMaxSpikeAmps = 40.0f;
            public static final int kSmartLimit = 40;
        }

        public static class AngleCurrentLimit {

            public static final double kLimitToAmps = 20.0f;
            public static final double kMaxSpikeTime = 25.0f;
            public static final double kMaxSpikeAmps = 20.0f;
            public static final int kSmartLimit = 20;
        }

        public static final double kDriveGearRatio = 1.0f / 8.14f;
        public static final double kAngleGearRatio = 1.0f / 12.8f;

        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumfrence = kWheelDiameter * Math.PI;

        public static final double kDrivePositionConversionFactor = kDriveGearRatio * kWheelCircumfrence;
        public static final double kDriveVelocityConverstionFactor = kDrivePositionConversionFactor / 60.0f;

        public static final double kAnglePositionConversionFactor = kAngleGearRatio * 360.0;
        public static final double kAngleVelocityConverstionFactor = kAnglePositionConversionFactor / 60.0f;

        public static final double kPAngle = 0.05; // AIR 0.01;
        public static final double kIAngle = 0; // AIR 0;
        public static final double kDAngle = 0.002; // AIR 0.0005;

        public static final double kPDrive = 0.2; //1.1;
        public static final double kIDrive = 0; // 0.0005; //0.0001;
        public static final double kDDrive = 3; //5;

        public static final double kSDrive = 0.375; // 0.375
        public static final double kVDrive = 2.5;
        public static final double kADrive = 0;

        public static final int kAngleCurrentLimit = 30;

        public static final CANSparkMax.IdleMode kDriveIdleMode = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode kAngleIdleMode = CANSparkMax.IdleMode.kCoast;

        public static final boolean angleMotorInverted = false;
        public static final boolean driveMotorInverted = false;
        public static final boolean KAbsoluteEncoderInverted = false;
    }

    public static class Modules {

        public static class FrontLeft {

            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(76.97476923465729));

            public static final SwerveModuleConfig FL0 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }

        public static class FrontRight {

            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(59.69142973423005));

            public static final SwerveModuleConfig FR1 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }

        public static class BackLeft {

            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(131.00274682044983));

            public static final SwerveModuleConfig BL2 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }

        public static class BackRight {

            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(170.9482741355896));

            public static final SwerveModuleConfig BR3 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }
    }

    public static final class SwerveMoveConsts {

        public static final float posTolerance = 0.011f;
        public static final float aTolerance = 0.5f;

        public static final float aVelocityTolerance = 1;
    }

    public static final class ClimberConsts {

        public static final int CLIMBER_LEFT = 50;
        public static final int CLIMBER_RIGHT = 51;

        public static final double kClimbSpeed = 0.5;
    }
}
