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



class Robot : public frc::IterativeRobot {
  static const int leftMotor1Lead = 1, leftMotor2 = 2, leftMotor3 = 3, rightMotor13Lead = 13, rightMotor12 = 12, rightMotor14 = 14, testMotor = 4;
  rev::CANSparkMax m_leftMotor1Lead{leftMotor1Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor13Lead{rightMotor13Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{leftMotor2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor3{leftMotor3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor12{rightMotor12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor14{rightMotor14, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_encoderleft = m_leftMotor1Lead.GetEncoder();
  rev::CANEncoder m_encoderright = m_rightMotor13Lead.GetEncoder();

  frc::DifferentialDrive m_robotDrive{m_leftMotor1Lead, m_rightMotor13Lead};

  frc::XboxController driver{1};

  frc::DoubleSolenoid *shift;

  //test motor
  rev::CANSparkMax m_testMotor{testMotor, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_testpidController = m_testMotor.GetPIDController();
  rev::CANEncoder m_testEncoder = m_testMotor.GetEncoder();

  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

  const double testMaxRPM = 5700;

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
  void EncoderVelocity();
  void LimeLight(char Item);
  void ColorSensor();
    
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

  bool testABtn;
  bool testBBtn;
  bool testXBtn;
  bool testYBtn;

  //Encoder Return Values
  double rightVelocity;
  double leftVelocity;
};
