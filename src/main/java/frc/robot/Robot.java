// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.RGB.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 *
 * TODO: Fix current limiting
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private double currentVeloc = 2;
    private RobotContainer m_robotContainer;
    private boolean preMatchLEDS = false;;
    int val = 0;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        CommandScheduler
            .getInstance()
            .onCommandInterrupt(command ->
                DriverStation.reportWarning("Interrupted Com:" + command.getName() + " Sub: " + command.getSubsystem(), false)
            );

        // new Thread(() -> {
        //     try {
        //         Thread.sleep(2000);
        //         sendAllianceRGB();
        //     } catch (Exception e) {}
        // })
        //     .start();
        preMatchLEDS = false;
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // SmartDashboard.putNumber("CPU Temp", RobotController.getCPUTemp());

        if(!preMatchLEDS && DriverStation.getAlliance().isPresent()){
            sendAllianceRGB();
            preMatchLEDS = true;
        }
        // RobotContainer.s_Swerve.getRobotRelativeSpeeds();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        RobotContainer.s_RGB.setLED(State.POPSICLE);
        RobotContainer.s_Pivot.setShooterCoastMode();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        RobotContainer.s_Pivot.setShooterBrakeMode();
        RobotContainer.resetModules.schedule();
        RobotContainer.s_RGB.setLED(State.RAINBOW);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        RobotContainer.s_Pivot.setShooterBrakeMode();

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        RobotContainer.resetModules.schedule();

        RobotContainer.s_Pivot.setShooterBrakeMode();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        //RobotContainer.s_Swerve.jogSingleModule(0, currentVeloc, false);
        RobotContainer.s_Swerve.jogAllModuleDrive(currentVeloc);
        currentVeloc *= -1;
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    public void sendAllianceRGB() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            RobotContainer.s_RGB.setLED(State.RED);
        } else {
            RobotContainer.s_RGB.setLED(State.BLUE);
        }
    }
}
