/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/IterativeRobot.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DoubleSolenoid.h>
#include "rev/CANSparkMax.h"
#include <frc/AnalogGyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include "WPILibVersion.h"
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <units/units.h>
#include <wpi/math>
#include <iostream>
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"


class Robot : public frc::IterativeRobot {
  //Drivetrain motors
  static const int leftMotor1Lead = 1, leftMotor2 = 2, leftMotor3 = 3, rightMotor13Lead = 13, rightMotor12 = 12, rightMotor14 = 14;
  rev::CANSparkMax m_leftMotor1Lead{leftMotor1Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor13Lead{rightMotor13Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{leftMotor2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor3{leftMotor3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor12{rightMotor12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor14{rightMotor14, rev::CANSparkMax::MotorType::kBrushless};

  //Drivetrain Encoders and motors

  //right motor
  rev::CANEncoder m_encoderright = m_rightMotor13Lead.GetEncoder();
  rev::CANPIDController m_rightPIDcontroller = m_rightMotor13Lead.GetPIDController();
  double rightkP = 6e-5, rightkI = 1e-6, rightkD = 0, rightkIz = 0, rightkFF = 0.000015, rightkMaxOutput = 1.0, rightkMinOutput = -1.0;

  //left motor
  rev::CANEncoder m_encoderleft = m_leftMotor1Lead.GetEncoder();
  rev::CANPIDController m_leftPIDcontroller = m_leftMotor1Lead.GetPIDController();
  double leftkP = 6e-5, leftkI = 1e-6, leftkD = 0, leftkIz = 0, leftkFF = 0.000015, leftkMaxOutput = 1.0, leftkMinOutput = -1.0;

  frc::DifferentialDrive m_robotDrive{m_leftMotor1Lead, m_rightMotor13Lead};

  //frc::Rotation2d gyroAngle{units::degree_t(-m_gyro.GetAngle())};

  //frc::Joystick m_driveStick{0};
  frc::XboxController driver{1};
  frc::XboxController operater{2};

  frc::DoubleSolenoid *shift;
  AHRS *ahrs;

  //test motor
  rev::CANSparkMax m_testMotor{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_testpidController = m_testMotor.GetPIDController();
  rev::CANEncoder m_testEncoder = m_testMotor.GetEncoder();  
  double testkP = 0.000400, testkI = 0.4, testkD = 0, testkIz = 0, testkFF = 0.000015, testkMaxOutput = .85, testkMinOutput = -.85;

  //Color Sensor Motor and Encoder
  rev::CANSparkMax m_colorwheel{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_colorPIDcontroller = m_colorwheel.GetPIDController();
  rev::CANEncoder m_colorencoder = m_colorwheel.GetEncoder();
  double colorkP = 6e-5, colorkI = 1e-6, colorkD = 0, colorkIz = 0, colorkFF = 0.000015, colorkMaxOutput = 0.8, colorkMinOutput = -0.8;

  //Shooter Motors
  rev::CANSparkMax m_shooterMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax m_shooterMotorOne{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_shooterPIDcontroller = m_shooterMotor.GetPIDController();
  rev::CANEncoder m_shooterencoder = m_shooterMotor.GetEncoder();
  double shooterkP = 6e-5, shooterkI = 1e-6, shooterkD = 0, shooterkIz = 0, shooterkFF = 0.000015, shooterkMaxOutput = 0.8, shooterkMinOutput = -0.8;

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  static constexpr frc::Color kBlueTarget = frc::Color(0.197, 0.464, 0.337);
  static constexpr frc::Color kGreenTarget = frc::Color(0.220, 0.512, 0.267);
  static constexpr frc::Color kRedTarget = frc::Color(0.324, 0.443, 0.231);
  static constexpr frc::Color kYellowTarget = frc::Color(0.280, 0.521, 0.198);

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void getInput();
  double AutoTargetTurn();
  void testPIDcontroller();
  void LimeLight(char Item);
  void colorPIDcontroller(double colorSetPoint);
  void readColorSensor();
  void executeColorSensor();
  void rightPIDcontroller(double rightSetPoint);
  void leftPIDcontroller(double leftSetPoint);
  void shooterPIDcontroller(double shooterSetPoint);

  //autonomous functions
  void forwardDrive(double feet, double speed);
  void rightTurn(double turnangleright);
  void leftTurn(double turnangleleft);
  void brakeRobot();
    
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc2::PIDController pid{1.0, 0.0, 0.0};

  //Controls
  double rotation; 
  double speed;
  double RT;
  double LT;
  double turnMultiplier;
  double driveMultiplier;
  bool throttle; 
  double throttleMultiplier;

  bool shiftUp;
  bool shiftDown;

  //Color Sensor
  bool testABtn;
  bool testBBtn;
  bool testXBtn;
  bool testYBtn;
  bool position;
  bool rotationControl;
  std::string gameData;
  frc::Color matchedColor;
  std::string currentColor;
  std::string initColor;
  std::string lastColor;
  int revs = 0;
  bool initp = true;
  bool initr = true;

  //Limelight and Autotargeting
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
  double steeringAdjust;
  double kF;
  double tP;
  double ty;
  bool autoAlign;
  bool nullTarget;
  double tCorrection;
  double targetDist;

  //test motor PID
  double testMaxRPM = 5700;
  double RPMstick;
  double testSetPoint;

};
