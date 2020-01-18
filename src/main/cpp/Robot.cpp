/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {
  //drive motors
  m_leftMotor1Lead.RestoreFactoryDefaults();
  m_rightMotor13Lead.RestoreFactoryDefaults();
  m_leftMotor2.RestoreFactoryDefaults();
  m_leftMotor3.RestoreFactoryDefaults();
  m_rightMotor12.RestoreFactoryDefaults();
  m_rightMotor14.RestoreFactoryDefaults();

  m_leftMotor2.Follow(m_leftMotor1Lead);
  m_leftMotor3.Follow(m_leftMotor1Lead);
  m_rightMotor12.Follow(m_rightMotor13Lead);
  m_rightMotor14.Follow(m_rightMotor13Lead);

  shift = new frc::DoubleSolenoid(1, 0);

  //test motor
  m_testMotor.RestoreFactoryDefaults();

  // set PID coefficients
  m_testpidController.SetP(kP);
  m_testpidController.SetI(kI);
  m_testpidController.SetD(kD);
  m_testpidController.SetIZone(kIz);
  m_testpidController.SetFF(kFF);
  m_testpidController.SetOutputRange(kMinOutput, kMaxOutput);

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("I Zone", kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", kFF);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  tP = frc::SmartDashboard::GetNumber("tP", -0.025);
  kF = frc::SmartDashboard::GetNumber("fP", 0.05);
  
  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts",0.0);
  frc::SmartDashboard::PutNumber("ts", ts);

  
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  shift->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Robot::TeleopPeriodic() {
  getInput();
	
  EncoderVelocity();

  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { m_testpidController.SetP(p); kP = p; }
  if((i != kI)) { m_testpidController.SetI(i); kI = i; }
  if((d != kD)) { m_testpidController.SetD(d); kD = d; }
  if((iz != kIz)) { m_testpidController.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { m_testpidController.SetFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { 
    m_testpidController.SetOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max; 
  }
  // read setpoint from joystick and scale by max rpm
  double SetPoint = 0.0;// = MaxRPM*m_stick.GetY();

  if(testABtn){
    SetPoint = 2000;
  }
  else if(testBBtn){
    SetPoint = 3000;
  }
  else if(testYBtn){
    SetPoint = 4000;
  }
  else if(testXBtn){
    SetPoint = 5000;
  }
  else{
    SetPoint = 0;
  }

  m_testpidController.SetReference(SetPoint, rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_testEncoder.GetVelocity());
}

void Robot::TestPeriodic() {}

void Robot::getInput() {
  rotation = driver.GetX(frc::GenericHID::JoystickHand::kRightHand); //setting controller x value to turn value
  //speed = m_driveStick.GetY();
  //rotation = m_driveStick.GetX();
  RT = driver.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand);  //setting right trigger to forward    
  LT = driver.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);  //setting left trigger to backward    
  speed = RT - LT;
  //throttle = m_driveStick.GetRawButton(1);  //used for halving speed
  throttle = driver.GetRawButton(6); //right bumper throttles speed

  //shiftDown = m_driveStick.GetRawButton(2); //shift up and down buttons
  //shiftUp = m_driveStick.GetRawButton(3);

  /*
  shiftDown = driver.GetBButton();
  shiftUp = driver.GetAButton();
  */

  //autoAlign = m_driveStick.GetRawButton(4);
  //nullTarget = m_driveStick.GetRawButton(5);

  autoAlign = driver.GetStartButton();
  nullTarget = driver.GetRawButton(10);

  //test motor
  testABtn = driver.GetAButton();
  testBBtn = driver.GetBButton();
  testXBtn = driver.GetXButton();
  testYBtn = driver.GetYButton();
}

void Robot::EncoderVelocity(){
  rightVelocity = m_encoderright.GetVelocity();
  leftVelocity = m_encoderleft.GetVelocity();
  frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoderright.GetVelocity());
  frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoderleft.GetVelocity());
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
