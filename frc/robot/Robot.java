// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//To comment out a block of code:
// select the block of code and press "ctrl + /"

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";

  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  boolean autoBalanceXMode;

  AHRS ahrs;

  // speed controlls for gyroscope in autonomous mode
  double leftSlow = 0.24;
  double rightSlow = -0.24;
  double rotateSpeed = 0.35;
  double rotateSpeedSlow = 0.25;
  // double rightFast = -1.1;

  // Encoder declaration
  // Encoder encoder = new Encoder(0, 1);
  // Inputs
  // AnalogGyro gyro = new AnalogGyro(0); // ANA Ch. 0

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftSpark = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax driveRightSpark = new CANSparkMax(2, MotorType.kBrushed);
  VictorSP driveLeftVictor = new VictorSP(1);
  VictorSP driveRightVictor = new VictorSP(0);

  // MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftSpark,
  // driveLeftVictor);
  // MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightSpark,
  // driveRightVictor);

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushed);
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushed);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  Joystick j = new Joystick(0);
  {
    try {
      /***********************************************************************
       * navX-MXP:
       * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
       * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro:
       * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
       * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  }

  MotorControllerGroup leftSide;
  MotorControllerGroup rightSide;

  DifferentialDrive drive;
  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 40;

  /**
   * \
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 3;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 40;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 0.5;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * 
   * 
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 3.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -2;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // encoder.setDistancePerPulse(1. / 256.);

    // gyro.reset();

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(true);
    driveLeftSpark.setIdleMode(IdleMode.kCoast);
    driveLeftVictor.setInverted(true);
    driveRightSpark.setInverted(true);
    driveRightSpark.setIdleMode(IdleMode.kCoast);
    driveRightVictor.setInverted(false);

    leftSide = new MotorControllerGroup(driveLeftSpark, driveLeftVictor);
    rightSide = new MotorControllerGroup(driveRightSpark, driveRightVictor);
    drive = new DifferentialDrive(leftSide, rightSide);



    
    /*
     * m
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.restoreFactoryDefaults();
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.restoreFactoryDefaults();
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.

    drive.tankDrive(left, right);
    // drive.arcadeDrive(forward, turn);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.setVoltage(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)",
        arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps    current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)",
        intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)",
        intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    // SmartDashboard.putNumber("Gyro angle:", Math.round(gyro.getAngle()));

  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    // driveLeftVictor.setNeutralMode(NeutralMode.Brake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    // driveRightVictor.setNeutralMode(NeutralMode.Brake);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == kConeAuto) {
      autonomousIntakePower = INTAKE_OUTPUT_POWER;

    } else if (m_autoSelected == kCubeAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  } 

  boolean foundChargingStation = false;

  @Override
  public void autonomousPeriodic() {

    
    if (Timer.getFPGATimestamp() - autonomousStartTime <= 2.5) {
      setDriveMotors(-0.7, 0);
    } else {
      setDriveMotors(0, 0);
    }

    // autoGyro();

    // 

  }

  // Sample Code for gyroscope to balance the robot
  final int DEGREES_THRESHOLD = 6;

  public void autoGyro() {
    if (Timer.getFPGATimestamp() - autonomousStartTime < 0.25) {
      return;
    }

    if (!foundChargingStation) {
      
      setDriveMotors(-0.7, 0);

      double angle = ahrs.getPitch();

      if (Math.abs(angle) > DEGREES_THRESHOLD) {
        foundChargingStation = true;

        autonomousStartTime = Timer.getFPGATimestamp();
      }

    } else {

      double angle = ahrs.getPitch();

      if (angle > DEGREES_THRESHOLD) {
        setDriveMotors(-0.11, 0);
      } else if (angle < -DEGREES_THRESHOLD) {
        setDriveMotors(0.11, 0);
      } else {
        setDriveMotors(0, 0);
      }
    }


  }
  //

  // private void set(double d) {}

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */

  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    // driveLeftVictor.setNeutralMode(NeutralMode.Brake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    // driveRightVictor.setNeutralMode(NeutralMode.Brake);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {

    // Encoder encoder = new Encoder(0, 1);
    double armPower;
    if (j.getRawButton(5)) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
      System.out.println("Arm lowering");

    } else if (j.getRawButton(6)) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
      System.out.println("Arm Raising");
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    setArmMotor(armPower);

    double intakePower;
    int intakeAmps;
    if (j.getRawAxis(2) > 0) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
      System.out.println("Out take");
    } else if (j.getRawAxis(3) > 0) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
      System.out.println("In Take");
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);

    /*
     * ;
     * 
     * 
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */

    double x = j.getRawAxis(1);
    double y = j.getRawAxis(4);
    if (Math.abs(x) < 0.05) {
      x = 0;
    }
    if (Math.abs(y) < 0.05) {
      y = 0;
    }
    setDriveMotors(x * 0.85, y * 0.85);

    // encoder.close();

  }
}
