// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc2024;

import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.team5817.frc2024.loops.Looper;
import com.team5817.frc2024.subsystems.Drive;
import com.team5817.frc2024.subsystems.Superstructure;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends LoggedRobot {

    SubsystemManager mSubsystemManager;
    Superstructure s = Superstructure.getInstance();
    Drive drive = Drive.getInstance();
    // public LoggedDashboardChooser<AutoBase> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
    private final Looper mEnabledLooper = new Looper();
  
  // HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
    @Override
    public void robotInit() {
      DriverStation.silenceJoystickConnectionWarning(true);
      // autos.put("Middle 4", new M6());
  
  
      DriverStation.startDataLog(DataLogManager.getLog());
  
      RobotState.getInstance().resetKalman();
      // for(HashMap.Entry<String, AutoBase> entry : autos.entrySet()) {
        // String N = entry.getKey();
        // AutoBase A = entry.getValue();
        // autoChooser.addOption(N, A);
      // }
  
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
      drive.resetModulesToAbsolute();
      mSubsystemManager = SubsystemManager.getInstance();
  
      mSubsystemManager.setSubsystems(
          Drive.getInstance(),
          Superstructure.getInstance() 
          );
          mSubsystemManager.registerEnabledLoops(mEnabledLooper);
          mEnabledLooper.start();
      }
  
    @Override
    public void robotPeriodic() {
      // auto = autoChooser.get();
      
      mEnabledLooper.outputToSmartDashboard();
          mSubsystemManager.outputLoopTimes();
      SubsystemManager.getInstance().outputTelemetry();
     OdometryLimeLight.getInstance().readInputsAndAddVisionUpdate();
    }
  
  
boolean disableGyroReset = false;
    @Override
    public void autonomousInit() {
      disableGyroReset = true;
      swerve = SwerveDrive.getInstance();
      swerve.zeroModules();
      SuperStructure.getInstance().setState(SuperState.AUTO);
      autoExecuter.setAuto(auto);
      autoExecuter.start();
    }
  
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() { 
    }
  
    /** This function is called once when teleop is enabled. */  
    @Override
    public void teleopInit() {
      
      swerve = SwerveDrive.getInstance();
      // swerve.fieldzeroSwerve();
      swerve.zeroModules();
  
    }
  
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
      controls.update();
    }
  
    /** This function is called once when the robot is disabled. */
  
    @Override
    public void disabledInit() {
      mSubsystemManager.stop();
      SuperStructure.getInstance().clearQueues();
      autoExecuter.stop();
      autoExecuter = new AutoExecuter();
    }
  
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
      RobotState.getInstance().outputTelemetry();
      if(!disableGyroReset)
        swerve.resetGryo(OdometryLimeLight.getInstance().getMovingAverageHeading());
        Logger.recordOutput("reset angle", OdometryLimeLight.getInstance().getMovingAverageHeading());
    }
  
    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }
  
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
      
  
    }
  }
 