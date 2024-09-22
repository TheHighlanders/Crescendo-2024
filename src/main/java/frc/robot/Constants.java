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
    public static final boolean diagnosticMode = true;

    public static class SwerveConst {

        public static final boolean kOpenLoop = true;

        public static final double kTranslateP = 1;
        public static final double kTranslateI = 0;
        public static final double kTranslateD = 0.1;

        public static final double kRotateP = 2;
        public static final double kRotateI = 0.0;
        public static final double kRotateD = 0;

        public static final double kMaxSpeedTele = 3.0; //Meters per Second
        public static final double kMaxAngularSpeedFast = Math.PI; //Degrees per Second

        public static final double kStickDeadband = 0.01;

        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        public static final double kWheelBase = Units.inchesToMeters(20.5);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        );

        public static final double speedLimit = 3.0;
        public static final double slowSpeedLimit = 0.5;

        public static final double accelerationLimit = 1.5;
        public static final double slowAccelerationLimit = 0.5;

        public static final double angularVelocityLimit = 180.0;
        public static final double slowAngularVelocityLimit = 45.0;
    }

    public static class Autonomous {

        public static final HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1.1, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.35, 0.00, 0.00), // Rotation PID constants //0.004 0.01 0.01
            2, // Max module speed, in m/s //used to be 1, changed to rotate faster AND IT WORKS!
            0.38615, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
    }

    public static class VisionConstants {

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 1);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1, 1, 1); // VecBuilder.fill(0.00040, 0.00080, 0.00050);
        //TODO: Actual stddevs

        public static final Translation2d kBlueSpeaker = new Translation2d(-0.04 + Units.inchesToMeters(6), 5.55);
        public static final Translation2d kRedSpeaker = new Translation2d(16.58 - Units.inchesToMeters(6), 5.55);

        public static final Translation3d robotCameraTranslation0 = new Translation3d(0.23, 0.288, 0.259); //-x, -y, z
        public static final Rotation3d robotCameraRotation0 = new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(25));
        public static final Transform3d kRobotCamera0 = new Transform3d(robotCameraTranslation0, robotCameraRotation0);

        public static final Translation3d robotCameraTranslation1 = new Translation3d(0.233, -0.288, 0.259); //-x, -y, z
        public static final Rotation3d robotCameraRotation1 = new Rotation3d(0, Units.degreesToRadians(25), -Units.degreesToRadians(25));
        public static final Transform3d kRobotCamera1 = new Transform3d(robotCameraTranslation1, robotCameraRotation1);
    }

    public static class Module {

        public static class DriveCurrentLimit {

            public static final double kLimitToAmps = 30.0f;
            public static final double kMaxSpikeTime = 20.0f;
            public static final double kMaxSpikeAmps = 35.0f;
            public static final int kSmartLimit = 35;
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

            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(347.1482491493225));

            public static final SwerveModuleConfig FL0 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }

        public static class FrontRight {

            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(307));

            public static final SwerveModuleConfig FR1 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }

        public static class BackLeft {

            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(57.84718573093414+180.0));

            public static final SwerveModuleConfig BL2 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }

        public static class BackRight {

            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;

            public static final Rotation2d absoluteEncoderOffset = new Rotation2d(Math.toRadians(256.65536928176877));

            public static final SwerveModuleConfig BR3 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset);
        }
    }

    public static final class SwerveMoveConsts {

        public static final float posPosTolerance = 0.05f;
        public static final float posVelTolerance = 0.1f;
        public static final float aPosTolerance = 5f;
        public static final float aVelTolerance = 2f;

        public static final float aVelocityTolerance = 1;
    }

    public static class Shooter {

        public static double kWaitTimeBeforeStop = 2; //seconds

        public static final int bottomFlywheelMotorID = 51;
        public static final int topFlywheelMotorID = 52;
        public static final double kBottomGearRatio = (1 / 1.0f);
        public static final double kBottomVelocityConversionFactor = kBottomGearRatio;
        public static final double kTopRatio = (1 / 1.0f);
        public static final int slotID = 0;

        public static final double velocityTolerance = 50;

        public static final int kCurrentLimit = 20;

        public static class ShooterCurrentLimit {

            public static final double kLimitToAmps = 5.0f;
            public static final double kMaxSpikeTime = 5.0f;
            public static final double kMaxSpikeAmps = 20;
            public static final int kSmartLimit = 10;
        }

        public static final int kShooterBeamBreakDIOPin = 0;

        public static class PIDValues {

            public static final double minOut = 0;
            public static final double maxOut = 1;
            public static final double kP = 0.001d;
            public static final double kI = 0d;
            public static final double kD = 0d;
            public static final double iMaxAccum = .6d;
        }

        public static class Pivot {

            public static final int SHOOTER = 41;
            public static final int slotID = 0;

            public static final boolean isInversed = true;
            public static final int inversionFactor = (isInversed ? -1 : 1);
            public static final int kAbsolDutyCycleDIOPin = 2;
            public static final double absoluteEncoderOffset = -185;

            public static final double initExtension = 13.231982231140137; // Inches
            public static final double readyInches = 13.8; // Inches

            public static final double shooterPivotRatio = 1 / 7.0f;

            public static final float shooterExtensionDeadzone = 0.125f;

            public static final double shooterBaseToArmPivotAxis = 7.4353; //inches

            public static class PIDValues {

                public static final double minOut = -0.25;
                public static final double maxOut = 1;
                public static final double kP = 1.4d;
                public static final double kI = 0.0d; //.005d;
                public static final double kD = 0.01; //30;
                public static final double iMaxAccum = 25d;
            }

            public static class ArmCurrentLimit {

                public static final double kLimitToAmps = 40.0f;
                public static final double kMaxSpikeTime = 25.0f;
                public static final double kMaxSpikeAmps = 40.0f;
                public static final int kSmartLimit = 40;
            }

            public static class actuatorConst {

                public static final double extensionVelocityDeadzone = 0.5;
                public static final double inchesToRotationsConversion = Units.metersToInches(0.012) * shooterPivotRatio;
            }
        }
    }

    public static class Intake {

        public static final int INTAKE = 53;
        public static final int slotID = 0;

        public static final int kIntakeBeamBreakDIOPin = 1;

        public static class IntakeCurrentLimit {

            public static final double kLimitToAmps = 40.0f;
            public static final double kMaxSpikeTime = 25.0f;
            public static final double kMaxSpikeAmps = 40.0f;
            public static final int kSmartLimit = 40;
        }

        public static class Pivot {

            public static final int INTAKE = 42;
            public static final int slotID = 0;

            public static final boolean isInversed = false;
            public static final int inversionFactor = (isInversed ? -1 : 1);
            public static final int kAbsolDutyCycleDIOPin = 3;
            public static final double absoluteEncoderOffset = 337d;

            public static final double intakeAtAmp = -8.5; 

            public static final double intakePivotRatio = 1;
            public static final double intakeInit = -13.2; // -12.5 for no grindy intake
            public static final float intakeAngleDeadzone = 0.5f; 
            public static final float intakeVelocityDeadzone = 0.1f;

            public static class PIDValues {

                public static final double minOut = -0.3;
                public static final double maxOut = 0.3;
                public static final double kP = 0.03d;
                public static final double kI = 0d;
                public static final double kD = 0d;
                public static final double kMaxI = 0d;
                public static final double iMaxAccum = 0d;

                public static class deviationPID {

                    public static final double kP = 0.0045d;
                    public static final double kI = 0d;
                    public static final double kD = 0d;
                    public static final double posTolerance = 1d;
                    public static final double velTolerance = 0.2d;
                }
            }

            public static class ArmCurrentLimit {

                public static final double kLimitToAmps = 40.0f;
                public static final double kMaxSpikeTime = 25.0f;
                public static final double kMaxSpikeAmps = 40.0f;
                public static final int kSmartLimit = 40;
            }
        }
    }

    public static final class ClimberConsts {

        public static final int CLIMBER_LEFT = 61;
        public static final int CLIMBER_RIGHT = 62;

        public static final int kServoRightID = 3;
        public static final int kServoLeftID = 4;

        public static final double kClimbSpeed = 0.45;

        public static final double kClimberP = 0.0;
        public static final double kClimberPrimePoint = 0.0;
    }

    public static final class Notes {

        public static final Translation2d AmpClose = new Translation2d(2.6, 7.0104);
        public static final Translation2d MidClose = new Translation2d(2.6, 5.5626);
        public static final Translation2d SourceClose = new Translation2d(2.6, 4.1148);
    }

    public static final class Points {

        public static final Translation2d shootAndLeaveS1 = new Translation2d(2, 2);
        public static final Translation2d shootAndLeaveS2 = new Translation2d(4, 2);
        public static final Translation2d shootAndLeaveM1 = new Translation2d(4, 5.5);

        public static final Translation2d shootAndLeaveA1 = new Translation2d(4, 7);
    }
}
