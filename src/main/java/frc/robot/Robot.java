// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandCoralPivot;
import frc.robot.subsystems.CommandElevator;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final TalonFX pivot;

  private static SparkMax elevator;

  Timer timer;
  
  

  public Robot() {
    m_robotContainer = new RobotContainer();

    elevator = new SparkMax(14, MotorType.kBrushless);
    pivot = new TalonFX(13);

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
    

    // CommandCoralPivot.PivotMotor.moveToSetPositionCommand(20);
  }

  @Override
  public void autonomousPeriodic() {
    if(timer.get() >= .5 && timer.get() < .95) pivot.set(.5);
    if(timer.get() > 0 && timer.get() < .5) elevator.set(.5);
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
    SmartDashboard.putString("Pivot Position", String.valueOf(pivot.getPosition().getValueAsDouble()));
    SmartDashboard.putString("Elevator Position", String.valueOf(m_robotContainer.elevator.getElevatorPosition()));
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