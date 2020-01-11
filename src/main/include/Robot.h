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
  static const int leftMotor1Lead = 1, leftMotor2 = 2, leftMotor3 = 3, rightMotor13Lead = 13, rightMotor12 = 12, rightMotor14 = 14;
  rev::CANSparkMax m_leftMotor1Lead{leftMotor1Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor13Lead{rightMotor13Lead, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{leftMotor2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor3{leftMotor3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor12{rightMotor12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor14{rightMotor14, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_encoderleft = m_leftMotor1Lead.GetEncoder();
  rev::CANEncoder m_encoderright = m_rightMotor13Lead.GetEncoder();

  frc::DifferentialDrive m_robotDrive{m_leftMotor1Lead, m_rightMotor13Lead};

  //frc::Rotation2d gyroAngle{units::degree_t(-m_gyro.GetAngle())};

  //frc::Joystick m_driveStick{0};
  frc::XboxController driver{1};

  frc::DoubleSolenoid *shift;
  AHRS *ahrs;

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
  
  //AHRS *ahrs = new AHRS(SPI::Port::kMXP);
  
  Robot(){
    try {
              /* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
              /* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
              /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
      ahrs = new AHRS(SPI::Port::kMXP);
    } catch (std::exception ex ) {
      std::string err_string = "Error instantiating navX-MXP:  ";
      err_string += ex.what();
      //DriverStation::ReportError(err_string.c_str());
    }
  }
    

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

  //Limelight and Autotargeting
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
  double steeringAdjust;
  double kF;
  double tP;
  bool autoAlign;
  bool nullTarget;
  double tCorrection;

  //Encoder Return Values
  double rightVelocity;
  double leftVelocity;

  //Kinematics and Odometry
  frc::DifferentialDriveKinematics kinematics{27.25_in};
  frc::DifferentialDriveOdometry m_odometry{ahrs->GetAngleAdjustment,frc::Pose2d{5_m, 13.5_m, 0_rad}};
};
