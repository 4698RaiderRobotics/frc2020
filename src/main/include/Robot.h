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
#include <frc/GenericHID.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include "rev/CANSparkMax.h"
#include <frc/AnalogGyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include "WPILibVersion.h"
#include <frc/SPI.h>
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <units/units.h>
#include <ctre/phoenix/music/Orchestra.h>
#include <wpi/math>
#include <iostream>
#include <frc/util/color.h>
#include <frc/AnalogInput.h>
#include <cameraserver/CameraServer.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "ctre/Phoenix.h"
#include "colorPIDcontroller.h"
#include "shooterPIDcontroller.h"
#include "readColorSensor.h"
#include "indexing.h"


class Robot : public frc::IterativeRobot {
  //frc::Rotation2d gyroAngle{units::degree_t(-m_gyro.GetAngle())};

  //Contoller declaration
  //frc::Joystick m_driveStick{0};
  frc::XboxController driver{1};
  frc::XboxController operater{2};

  //Pnuematics Piston Declaration
  frc::DoubleSolenoid *shift;
  frc::DoubleSolenoid *intake;
  frc::DoubleSolenoid *boost;

  //Gyro Declaration
  AHRS *ahrs;

  //timer declaration
  frc::Timer m_waitTimer;

  //distance sensor
  frc::AnalogInput index1{3};

  //All Motors

  //Drivetrain motors
  static const int leftMotor1Lead = 1, leftMotor2 = 2, leftMotor3 = 3, rightMotor13Lead = 13, rightMotor12 = 12, rightMotor14 = 14;
  rev::CANSparkMax m_leftMotor1Lead{leftMotor1Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor13Lead{rightMotor13Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{leftMotor2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor3{leftMotor3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor12{rightMotor12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor14{rightMotor14, rev::CANSparkMax::MotorType::kBrushless};

  frc::DifferentialDrive m_robotDrive{m_leftMotor1Lead, m_rightMotor13Lead};

  //Shooter Motors
  rev::CANSparkMax m_shooterMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  //test motor CAN ID 5

  //declaring talon variables
  TalonSRX tsrx1 = /*device ID*/{6};
  TalonSRX tsrx2 = /*device ID*/{7};  

  //Victor spx CAN ID 8
  
  //color motor CAN ID 9

  //Intake Motor
  VictorSPX vspx2 = /*device ID*/{10};

  /*
  Current CAN IDs
  left drive motors 1, 2, 3
  shooter motors 4, 5
  climb motors 6, 7
  Indexing 8
  Color Wheel 9
  Intake motor 10
  right drive motors 12, 13, 14
  */

  //Encoders

  //right motor
  rev::CANEncoder m_encoderright = m_rightMotor13Lead.GetEncoder();
  rev::CANPIDController m_rightPIDcontroller = m_rightMotor13Lead.GetPIDController();
  double rightkP = 6e-5, rightkI = 1e-6, rightkD = 0, rightkIz = 0, rightkFF = 0.000015, rightkMaxOutput = 1.0, rightkMinOutput = -1.0;

  //left motor
  rev::CANEncoder m_encoderleft = m_leftMotor1Lead.GetEncoder();
  rev::CANPIDController m_leftPIDcontroller = m_leftMotor1Lead.GetPIDController();
  double leftkP = 6e-5, leftkI = 1e-6, leftkD = 0, leftkIz = 0, leftkFF = 0.000015, leftkMaxOutput = 1.0, leftkMinOutput = -1.0;

  //frc::Rotation2d gyroAngle{units::degree_t(-m_gyro.GetAngle())};

  //Limit Switches
  //climber left
  frc::DigitalInput climberLeftSwitch{0};
  //climber right
  frc::DigitalInput climberRightSwitch{1};
  
  frc::DigitalInput thirdSwitch{2};


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
  void LimeLight(char Item);
  void rightPIDcontroller(double rightSetPoint);
  void leftPIDcontroller(double leftSetPoint);
  void rampUpSpeed(double time, double current);
  void executeColorSensor();

  //autonomous functions
  void forwardDrive(double feet, double speed);
  void backwardDrive(double feet, double speed);
  void ccwTurn(double turnangleright);
  void cwTurn(double turnangleleft);
  void brakeRobot();
    
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  
  frc2::PIDController pid{1.0, 0.0, 0.0};

  //Driver Controls
  double rotation; 
  double speed;
  double RT;
  double LT;
  double turnMultiplier;
  double driveMultiplier;
  bool throttle; 
  double throttleMultiplier;

  bool enableclimb;
  bool climbmode;
  double leftclimbinput;
  double rightclimbinput;

  bool shiftbuttonpressed;
  bool shiftup;

  bool boostbuttonpressed;
  bool boostup;

  //Operater Controls
  bool shooterThrottle;
  bool shooterspit;
  bool moveBall;


  bool intakeUse;
  bool intakebuttonpressed;

  bool indexbutton;
  bool indexup;
  bool reverseindexbutton;
  bool indextoggle;
  bool indexauto;

  double intakeBall;

  double index3volt;
  bool thirdInput;

  //Limelight and Autotargeting
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
  double steeringAdjust;
  double tx;
  double kF;
  double tP;
  double ty;
  bool autoAlign;
  bool nullTarget;
  double tCorrection;
  double targetDist;
  bool targeting;
  double distright;
  double distleft;
  double povsetpoints;

  bool intakeup = true;

  //autonomous bools
  bool driveForward;
  bool turnRight;
  bool turnLeft;
  bool targetAuto;
  bool targetAuto2;
  bool autoShoot;
  bool autoShoot2;
  int autocounter;
  double initautoangle;
  bool recenterbot;
  bool autointakedown;
  bool driveForward2;
  bool driveBackward;
  bool autointakeup;
  bool shiftGear;
  bool shiftuponce;

  //Climber
  bool climberLeftInput;
  bool climberRightInput;
};