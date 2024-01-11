package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Module;
import frc.robot.Constants.SwerveConst;
import frc.robot.util.CANSparkMaxCurrent;
import frc.robot.util.SwerveModuleConfig;

public class SwerveModule {
    public CANSparkMaxCurrent angleMotor;
    public CANSparkMaxCurrent driveMotor;

    public int moduleNumber;

    public SparkPIDController driveController;
    public SparkPIDController angleController;

    private SimpleMotorFeedforward driveFeedforward;

    public RelativeEncoder driveEncoder;
    public RelativeEncoder angleEncoder;
    public double angleReference;
    public double driveReference;

    public SparkAbsoluteEncoder absoluteEncoder;

    private Rotation2d KModuleAbsoluteOffset;
    private Rotation2d lastAngle;

    public SwerveModule(int moduleNumber, SwerveModuleConfig config){
        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMaxCurrent(config.driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMaxCurrent(config.angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        
        driveController = driveMotor.getPIDController();
        angleController = angleMotor.getPIDController();
        
        /* Creates an additional FF controller for extra drive motor control */
        driveFeedforward = new SimpleMotorFeedforward(Module.kSDrive, Module.kVDrive, Module.kADrive);

        absoluteEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        this.KModuleAbsoluteOffset = config.absoluteEncoderOffset;

        configureDriveMotor();
        configureAngleMotor();
    }

    /**
     * Sets both Angle and Drive to desired states
     * 
     * @param state: Desired module state
     * @param isOpenLoop: Controls if the drive motor use a PID loop
     */
    public void setModuleState(SwerveModuleState state, boolean isOpenLoop){
        state = SwerveModuleState.optimize(state, getAnglePosition());
        
        setAngleState(state);
        setDriveState(state, isOpenLoop);
    }

    /**
     * Sets the Drive Motor to a desired state, 
     * if isOpenLoop is true, it will be set as a percent, if it is false, than it will use a velocity PIDF loop
     * 
     * @param state: Desired module state
     * @param isOpenLoop: Whether or not to use a PID loop
     */
    public void setDriveState(SwerveModuleState state, boolean isOpenLoop){
        if(isOpenLoop){
            double motorPercent = state.speedMetersPerSecond / SwerveConst.kMaxSpeedTele;
            driveMotor.set(motorPercent);
        } else {
            driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, 
                                                        driveFeedforward.calculate(state.speedMetersPerSecond));
            driveReference = state.speedMetersPerSecond;
        }
    }

    /**
     * Sets the Angle Motor to a desired state, does not set the state if speed is too low, to stop wheel jitter
     * 
     * @param state: Desired module state
     */
    public void setAngleState(SwerveModuleState state){
        // Anti Jitter Code, not sure if it works, need to test and review
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= SwerveConst.kMaxAngularSpeedFast * 0.001)
        ? lastAngle : state.angle;
        // Rotation2d angle = state.angle;
        if (angle != null){
            angleController.setReference(angle.getDegrees(), ControlType.kPosition);
            angleReference = angle.getDegrees();
        }
        // lastAngle = state.angle;
    }

    /**
     * Returns the position of the Angle Motor, measured with integrated encoder
     * 
     * @return Angle Motor Position
     */
    public Rotation2d getAnglePosition(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    /**
     * Returns the velocity of the Drive Motor, measured with integrated encoder
     * 
     * @return Drive Motor Velocity
     */
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the position of the Drive Motor, measured with integrated encoder
     * 
     * @return Drive Motor Position
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the position of the module using the absolute encoder
     * 
     * @return Position of the module between 0 and 360, as a Rotation2d
     */
    public Rotation2d getAbsolutePosition(){
        /* Gets Position from SparkMAX absol encoder * 360  to degrees */
        double positionDeg = absoluteEncoder.getPosition() * 360.0d;
        
        /*Subtracts magnetic offset to get wheel position */
        positionDeg -= KModuleAbsoluteOffset.getDegrees();

        /* Inverts if necesary */
        positionDeg *= (Module.KAbsoluteEncoderInverted ? -1 : 1);

        return Rotation2d.fromDegrees(positionDeg);
    }

    /**
     * 
     * @return Swerve Module Position (Position & Angle)
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getAnglePosition());
    }

    /**
     * 
     * @return Swerve Module State (Velocity & Angle)
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
    }

    /**
     *  Returns the assigned module number
     */
    public int getModuleNumber(){
        return moduleNumber;
    }

    /**
     * Configures Drive Motor using parameters from Constants
     */
    private void configureDriveMotor(){
        driveMotor.restoreFactoryDefaults();

        driveMotor.setInverted(Module.driveMotorInverted);
        driveMotor.setIdleMode(Module.kDriveIdleMode);
        
        /* Sets encoder ratios to actual module gear ratios */
        driveEncoder.setPositionConversionFactor(Module.kDrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(Module.kDriveVelocityConverstionFactor);

        /* Configures PID loop */
        driveController.setP(Module.kPDrive);
        driveController.setI(Module.kIDrive);
        driveController.setD(Module.kDDrive);

        // driveMotor.setSmartCurrentLimit(Module.kDriveCurrentLimit);
        driveMotor.setSpikeCurrentLimit(Module.DriveCurrentLimit.kLimitToAmps, Module.DriveCurrentLimit.kMaxSpikeTime, Module.DriveCurrentLimit.kMaxSpikeAmps, Module.DriveCurrentLimit.kSmartLimit);

        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    /**
     * Configures Angle Motor using parameters from Constants
     */
    private void configureAngleMotor(){
        angleMotor.restoreFactoryDefaults();

        angleMotor.setInverted(Module.angleMotorInverted);
        angleMotor.setIdleMode(Module.kAngleIdleMode);
        
        /* Sets encoder ratios to actual module gear ratios */
        angleEncoder.setPositionConversionFactor(Module.kAnglePositionConversionFactor);
        angleEncoder.setVelocityConversionFactor(Module.kAngleVelocityConverstionFactor);

        /* Configures PID loop */
        angleController.setP(Module.kPAngle);
        angleController.setI(Module.kIAngle);
        angleController.setD(Module.kDAngle);

        /* Defines wheel angles as -pi to pi */
        angleController.setPositionPIDWrappingMaxInput(180.0d);
        angleController.setPositionPIDWrappingMinInput(-180.0d);
        angleController.setPositionPIDWrappingEnabled(true);

        // angleMotor.setSmartCurrentLimit(Module.kAngleCurrentLimit);
        angleMotor.setSpikeCurrentLimit(Module.AngleCurrentLimit.kLimitToAmps, Module.AngleCurrentLimit.kMaxSpikeTime, Module.AngleCurrentLimit.kMaxSpikeAmps, Module.AngleCurrentLimit.kSmartLimit);


        angleMotor.burnFlash();

        setIntegratedAngleToAbsolute();
    }

    /**
     * Resets the Angle Motor to the position of the absolute position
     */
    public void setIntegratedAngleToAbsolute(){
        angleEncoder.setPosition(getAbsolutePosition().getDegrees());
    }

    public void runPeriodicLimiting(){
        driveMotor.periodicLimit();
        angleMotor.periodicLimit();
    }
}
