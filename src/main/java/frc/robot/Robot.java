// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;


  Timer timer;
  
  

  public Robot() {
    m_robotContainer = new RobotContainer();

    timer = new Timer();
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    timer.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    
  }

  @Override
  public void autonomousPeriodic() {
    // if(timer.get() > 0 && timer.get() < .2) {
    //   RobotContainer.coralPivotAuton();
    // }

    // else if(timer.get() >= .2 && timer.get() < 1) {
    //   RobotContainer.moveElevator();
    //   RobotContainer.coralPivotStop();
    // }

    // else {
    //   RobotContainer.coralPivotStop();
    // }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putString("Pivot Position", String.valueOf(pivot.getPosition().getValueAsDouble()));
    // SmartDashboard.putString("Elevator Position", String.valueOf(m_robotContainer.elevator.getElevatorPosition()));
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}