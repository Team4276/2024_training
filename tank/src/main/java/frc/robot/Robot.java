// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private VictorSP leftMotor;
  private VictorSP rightMotor;
  private XboxController myController;

  // Ha

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private boolean isConnected = false;
  private Rotation2d yawPosition = Rotation2d.fromDegrees(0.0);

  private Rotation2d deriredHeading = Rotation2d.fromDegrees(0.0);
  private double prevTurnLR = 0.0;
  private double prevHeadingError = 0.0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftMotor = new VictorSP(0);
    rightMotor = new VictorSP(1);
    myController = new XboxController(0);
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
    isConnected = gyro.isConnected();
    yawPosition = Rotation2d.fromDegrees(gyro.getAngle());

    if (isConnected) {
      SmartDashboard.putNumber("Gyro Yaw Radians", yawPosition.getRadians());
    } else {
      SmartDashboard.putNumber("Gyro Yaw Radians", -1.0);
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Arcade drive
    double turnLR = myController.getLeftX();
    double driveFB = myController.getRightY();

    double leftPower = driveFB;
    double rightPower = leftPower;

    leftPower += turnLR;
    rightPower -= turnLR;


    if (leftPower > 1.0) {
      leftPower = 1.0;
    } else if (leftPower < -1.0) {
      leftPower = -1.0;
    }

    rightPower -= turnLR;
    if (rightPower > 1.0) {
      rightPower = 1.0;
    } else if (rightPower < -1.0) {
      rightPower = -1.0;
    }

    leftMotor.set(leftPower);
    rightMotor.set(rightPower * -1.0);  // Motors face opposite directions, -1 makes both go forward for positive power

  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    leftMotor.set(0.0);
    rightMotor.set(0.0);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
