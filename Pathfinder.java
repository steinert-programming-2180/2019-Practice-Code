import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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
    /*
    DoubleSolenoid oneDoubleSolenoid = new DoubleSolenoid(0,1);
    DoubleSolenoid twoDoubleSolenoid = new DoubleSolenoid(2,3);
    DoubleSolenoid threeDoubleSolenoid = new DoubleSolenoid(4,5);
  
    Compressor airCompressor = new Compressor();
    */
    Joystick oneJoystick = new Joystick(0);
    JoystickButton pistonOut = new JoystickButton(oneJoystick, 1);
    JoystickButton pistonIn = new JoystickButton(oneJoystick, 2);

    UsbCamera camera = new UsbCamera("Cam0", 1);

    TalonSRX leftMotor1 = new TalonSRX(10);
    TalonSRX leftMotor2 = new TalonSRX(1);
    TalonSRX leftMotor3 = new TalonSRX(2);
    TalonSRX rightMotor1 = new TalonSRX(11);
    TalonSRX rightMotor2 = new TalonSRX(6);
    TalonSRX rightMotor3 = new TalonSRX(7);

    double wheelbase_width = 6.25;

    AHRS gyro = new AHRS(Port.kMXP);

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

      EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
      EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());

      double l;
      double r;

      double gyroHeading;
      double desiredHeading;

      double angleDifference;
      double turn;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer server = CameraServer.getInstance();
    server.startAutomaticCapture();

    camera.setFPS(30);
    camera.setResolution(640, 480);
    camera.setBrightness(50);
    //SmartDashboard.putData(value);

    gyro.reset();
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

      left.configureEncoder(leftMotor1.getSelectedSensorPosition(), 1024, 6.25);
      right.configureEncoder(rightMotor1.getSelectedSensorPosition(), 1024, 6.25);

      left.configurePIDVA(1.0, 0.0, 0.0, 1 / 5, 0);
      right.configurePIDVA(1.0, 0.0, 0.0, 1 / 5, 0);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    l = left.calculate(leftMotor1.getSelectedSensorPosition(0));
    r = right.calculate(rightMotor1.getSelectedSensorPosition(0));

    gyroHeading = gyro.getAngle();
    desiredHeading = Pathfinder.r2d(left.getHeading());

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

    rightMotor1.set(ControlMode.PercentOutput, rightVal);
    rightMotor2.set(ControlMode.PercentOutput, rightVal);
    rightMotor3.set(ControlMode.PercentOutput, rightVal);
    leftMotor1.set(ControlMode.PercentOutput, leftVal);
    leftMotor2.set(ControlMode.PercentOutput, leftVal);
    leftMotor3.set(ControlMode.PercentOutput, leftVal);
  }
