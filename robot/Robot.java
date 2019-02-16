/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.cscore.UsbCamera;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import com.kauailabs.navx.frc.AHRS;

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
  double wheelbase_width = 6.25;

  AHRS gyro = new AHRS(Port.kMXP);

  public static Potentiometer armPot = new AnalogPotentiometer(0,360,0);

  public static boolean sensorInPhase, motorInverted; 

  Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);

    Waypoint[] points = new Waypoint[] {
            new Waypoint(-4, -1, Pathfinder.d2r(-45)),
            new Waypoint(-2, -2, 0),
            new Waypoint(0, 0, 0)
    };

    Trajectory trajectory = Pathfinder.generate(points, config);

    TankModifier modifier = new TankModifier(trajectory).modify(6.25);

    Trajectory leftTrajectory = modifier.getLeftTrajectory();
    Trajectory rightTrajectory = modifier.getRightTrajectory();

    EncoderFollower leftfollower = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower rightfollower = new EncoderFollower(modifier.getRightTrajectory());

    double l;
    double r;

    double gyroHeading;
    double desiredHeading;

    double angleDifference;
    double turn;

  TalonSRX wristLeft = new TalonSRX(13);
  TalonSRX wristRight = new TalonSRX(14);
  TalonSRX intake = new TalonSRX(15);
  
  TalonSRX armLeft = new TalonSRX(11);
  TalonSRX armRight = new TalonSRX(12);

  TalonSRX left1 = new TalonSRX(01);
  TalonSRX left2 = new TalonSRX(02);
  TalonSRX left3 = new TalonSRX(03);

  TalonSRX[] left = {left1, left2, left3};

  TalonSRX right1 = new TalonSRX(04);
  TalonSRX right2 = new TalonSRX(05);
  TalonSRX right3 = new TalonSRX(06);

  TalonSRX LeftArm = new TalonSRX(11);
  TalonSRX RightArm = new TalonSRX(12);

  TalonSRX LeftWrist = new TalonSRX(13);
  TalonSRX RightWrist = new TalonSRX(14);
  
  TalonSRX Intake = new TalonSRX(15);

  TalonSRX[] right = {right1, right2, right3};

  Joystick stick = new Joystick(0);
  Joystick stick1 = new Joystick(1);
  Joystick stick2 = new Joystick(2);

  boolean foward = true;

  boolean previousButton = false;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());

    setupArmPID(0);
    setupWristPID(0);

    SmartDashboard.putData("Auto mode", m_chooser);
  }

  public void setMotors (TalonSRX[] left, TalonSRX[] right, double Lspeed, double Rspeed, boolean foward) {
    if (foward) {
      for (TalonSRX x : left) {
        x.set(ControlMode.PercentOutput, Lspeed);
      } for (TalonSRX x : right) {
        x.set(ControlMode.PercentOutput, -1 * Rspeed);
      }
    } else {
      for (TalonSRX x : right) {
        x.set(ControlMode.PercentOutput, Lspeed);
      } for (TalonSRX x : left) {
        x.set(ControlMode.PercentOutput, -1 * Rspeed);
      }
    }
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
    leftfollower.configureEncoder(left1.getSelectedSensorPosition(), 1024, 6.25);
    rightfollower.configureEncoder(right1.getSelectedSensorPosition(), 1024, 6.25);

    leftfollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 5, 0);
    rightfollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 5, 0);

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
    l = leftfollower.calculate(left1.getSelectedSensorPosition(0));
    r = rightfollower.calculate(right1.getSelectedSensorPosition(0));

    gyroHeading = gyro.getAngle();
    desiredHeading = Pathfinder.r2d(leftfollower.getHeading());

    angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
    turn = 0.03 * angleDifference; // 0.8 * (1.0/80.0)

    double leftVal = l + turn;
    double rightVal = r + turn;

    if (leftVal > 1.0) {
      leftVal = 1.0;
    } else if (leftVal < -1.0) {
      leftVal = -1.0;
    }

    if (rightVal > 1.0) {
      rightVal = 1.0;
    } else if (rightVal < -1.0) {
      rightVal = -1.0;
    }

    right1.set(ControlMode.PercentOutput, rightVal);
    right2.set(ControlMode.PercentOutput, rightVal);
    right3.set(ControlMode.PercentOutput, rightVal);
    left1.set(ControlMode.PercentOutput, leftVal);
    left2.set(ControlMode.PercentOutput, leftVal);
    left3.set(ControlMode.PercentOutput, leftVal);
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
    armRight.setNeutralMode(NeutralMode.Brake);
    armLeft.setNeutralMode(NeutralMode.Brake);
    wristRight.setNeutralMode(NeutralMode.Brake);
    wristLeft.setNeutralMode(NeutralMode.Brake);
    intake.setNeutralMode(NeutralMode.Brake);

    armLeft.setInverted(false);
    armRight.setInverted(false);

    

    wristLeft.setInverted(false);
    wristRight.setInverted(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //Basic manual drivetrain, with reversable stearing
    //--------------------------------------------
    if (!previousButton && stick.getRawButton(1)) {
      foward = !foward;
    }
    previousButton = stick.getRawButton(1);
    SmartDashboard.putBoolean("Button", stick.getRawButton(1));
    SmartDashboard.putBoolean("Fowards", foward);
    setMotors(left, right, -1 * stick.getRawAxis(1), -1 * stick1.getRawAxis(1), foward);
    SmartDashboard.putNumber("leftStick", stick.getRawAxis(1));

    SmartDashboard.putBoolean("armButton", stick.getRawButton(3));


    SmartDashboard.putNumber("ArmPot Position", armPot.get()); //Added by Nikhil, reports Pot position

    /*  This controls the arm and wrist manually, we'll probably change most of this
    if (stick2.getRawButton(3)) {
      armRight.set(ControlMode.PercentOutput, -0.75);
      armLeft.set(ControlMode.PercentOutput, 0.75);
    } else if (stick2.getRawButton(2)) {
      armRight.set(ControlMode.PercentOutput, 0.75);
      armLeft.set(ControlMode.PercentOutput, -0.75);
    } else if (stick2.getRawButton(6)) {
      armRight.set(ControlMode.PercentOutput, -0.15);
      armLeft.set(ControlMode.PercentOutput, 0.15);
    } else if (stick2.getRawButton(7)) {
      armRight.set(ControlMode.PercentOutput, 0.15);
      armLeft.set(ControlMode.PercentOutput, -0.15);
    } else {
      armRight.set(ControlMode.PercentOutput, 0);
      armLeft.set(ControlMode.PercentOutput, 0);
    }

    if (stick2.getRawButton(4)) {
      wristLeft.set(ControlMode.PercentOutput, 0.75);
      wristRight.set(ControlMode.PercentOutput, -0.75);
    } else if (stick2.getRawButton(5)) {
      wristLeft.set(ControlMode.PercentOutput, -0.75);
      wristRight.set(ControlMode.PercentOutput, 0.75);
    } else {
      wristLeft.set(ControlMode.PercentOutput, 0);
      wristRight.set(ControlMode.PercentOutput, 0);
    }

    */

    if(stick2.getRawButton(11)) { //Manually runs intake
      intake.set(ControlMode.PercentOutput, 1);
    } else if (stick2.getRawButton(10)) {
      intake.set(ControlMode.PercentOutput, -1);
    } else {
      intake.set(ControlMode.PercentOutput, 0);
    }

    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void setupArmPID(int pidSlot) {
    sensorInPhase = true;
    motorInverted = false;

    armRight.follow(armLeft);

    armLeft.setInverted(motorInverted);
    armRight.setInverted(motorInverted);
      
    armLeft.configSelectedFeedbackSensor(FeedbackDevice.Analog, pidSlot, 10);
		armLeft.setSensorPhase(sensorInPhase);
		
	  armLeft.configAllowableClosedloopError(pidSlot, Constants.maxArmError, 10);
		
		armLeft.configNominalOutputForward(0, 10);
		armLeft.configNominalOutputReverse(0, 10);
		armLeft.configPeakOutputForward(Constants.maxArmSpeed, 10);
		armLeft.configPeakOutputReverse(-Constants.maxArmSpeed, 10);
		
		armLeft.config_kF(0, 0.0, 10);
		armLeft.config_kP(0, Constants.armKP, 10);
		armLeft.config_kI(0, Constants.armKI, 10);
		armLeft.config_kD(0, Constants.armKD, 10);
  }

  public void setupWristPID(int pidSlot) {
    sensorInPhase = true;
    motorInverted = false;

    wristRight.follow(wristLeft);

    wristLeft.setInverted(motorInverted);
    wristLeft.setInverted(motorInverted);
    
    wristLeft.configSelectedFeedbackSensor(FeedbackDevice.Analog, pidSlot, 10);
    wristLeft.setSensorPhase(sensorInPhase);
  
    wristLeft.configAllowableClosedloopError(pidSlot, Constants.maxWristError, 10);
  
    wristLeft.configNominalOutputForward(0, 10);
    wristLeft.configNominalOutputReverse(0, 10);
    wristLeft.configPeakOutputForward(Constants.maxWristSpeed, 10);
    wristLeft.configPeakOutputReverse(-Constants.maxWristSpeed, 10);
  
    wristLeft.config_kF(0, 0.0, 10);
    wristLeft.config_kP(0, Constants.wristKP, 10);
    wristLeft.config_kI(0, Constants.wristKI, 10);
    wristLeft.config_kD(0, Constants.wristKD, 10);
  }
}
