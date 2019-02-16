/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  TalonSRX right1 = new TalonSRX(4);
  TalonSRX right2 = new TalonSRX(5);
  TalonSRX right3 = new TalonSRX(6);

  TalonSRX left1 = new TalonSRX(1);
  TalonSRX left2 = new TalonSRX(2);
  TalonSRX left3 = new TalonSRX(3);

  TalonSRX armLeft = new TalonSRX(11);
  TalonSRX armRight = new TalonSRX(12);
  Potentiometer armPot = new AnalogPotentiometer(0, 360 , 0);
  double startAngle;
  double endAngle = 105;
  double currAngle;
  double lastAngle;

  TalonSRX wristLeft = new TalonSRX(13);
  TalonSRX wristRight = new TalonSRX(14);

  TalonSRX intake = new TalonSRX(15);

  Joystick joy = new Joystick(0);
  Joystick joy2 = new Joystick(1);

  //AHRS gyro = new AHRS(serial_port_id, data_type, update_rate_hz));
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    startAngle = armPot.get();
    currAngle = startAngle;
    lastAngle = startAngle;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    double leftSpeed = -joy.getRawAxis(0) * .5;
    double rightSpeed = -joy2.getRawAxis(0) * .5;

    

    left1.set(ControlMode.PercentOutput, leftSpeed);
    left2.set(ControlMode.PercentOutput, leftSpeed);
    left3.set(ControlMode.PercentOutput, leftSpeed);

    right1.set(ControlMode.PercentOutput, rightSpeed);
    right2.set(ControlMode.PercentOutput, rightSpeed);
    right3.set(ControlMode.PercentOutput, rightSpeed);

    double speed = 1;
    double diff = 0;
    currAngle = armPot.get();
    if(currAngle > (startAngle + endAngle)/2 && currAngle > lastAngle){
      diff = endAngle - currAngle;
    }
    else if(currAngle < lastAngle){
      diff = currAngle-startAngle;
    }
    speed *= diff/100;

    SmartDashboard.putNumber("Start", startAngle);
    SmartDashboard.putNumber("End", endAngle);
    SmartDashboard.putNumber("Cur", currAngle);
    SmartDashboard.putNumber("Diff", diff);

    if(joy.getRawButton(3)){
      armLeft.set(ControlMode.PercentOutput, speed);
      armRight.set(ControlMode.PercentOutput, -speed);
    }
    
    if(joy.getRawButton(2)){
      armLeft.set(ControlMode.PercentOutput, -speed);
      armRight.set(ControlMode.PercentOutput, speed);
    }

    if(joy.getRawButtonReleased(3) || joy.getRawButtonReleased(2)){
      armLeft.set(ControlMode.PercentOutput, 0);
      armRight.set(ControlMode.PercentOutput, 0);
    }

    SmartDashboard.putNumber("Potentiomeyterer", armPot.get());

    if(joy2.getRawButton(3)){
      wristLeft.set(ControlMode.PercentOutput, 1);
      wristRight.set(ControlMode.PercentOutput, -1);
    }

    if(joy2.getRawButton(2)){
      wristLeft.set(ControlMode.PercentOutput, -1);
      wristRight.set(ControlMode.PercentOutput, 1);
    }

    if(joy2.getRawButtonReleased(3) || joy2.getRawButtonReleased(2)){
      wristLeft.set(ControlMode.PercentOutput, 0);
      wristRight.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
