/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {

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
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  shift->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Robot::TeleopPeriodic() {
  getInput();

  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts",0.0);
  frc::SmartDashboard::PutNumber("ts", ts);

  EncoderVelocity();


  if (shiftUp) {
		shift->Set(frc::DoubleSolenoid::Value::kReverse);
    bool gear = false;
	}
	if (shiftDown) {
		shift->Set(frc::DoubleSolenoid::Value::kForward);
    bool gear = true;
	}
  

  //Sets shifter to high or low gear

  if (throttle) {
    throttleMultiplier = .5;
  }
  else{
    throttleMultiplier = 1;
  }
  
  	//Limelight Target Reflective Strips
	if(autoAlign){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",0);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",0);
		table->PutNumber("pipeline", 1);
		tCorrection = AutoTargetTurn();
	}
	//Turns off Targeting
	else{
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);
		nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",1);
		tCorrection = 0;
	}
  //Makes the throttle button half the output
  turnMultiplier = .6;
  driveMultiplier = .8;
  m_robotDrive.ArcadeDrive(-speed*throttleMultiplier*driveMultiplier, (-rotation*throttleMultiplier*turnMultiplier)+tCorrection);  //Drive function
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

  shiftDown = driver.GetBButton();
  shiftUp = driver.GetAButton();


  //autoAlign = m_driveStick.GetRawButton(4);
  //nullTarget = m_driveStick.GetRawButton(5);

  autoAlign = driver.GetStartButton();
  nullTarget = driver.GetRawButton(10);
}

double Robot::AutoTargetTurn(){
  double tx = table->GetNumber("tx",0.0); 
  double ty = table->GetNumber("ty",0.0); 
  double ta = table->GetNumber("ta",0.0); 
  double ts = table->GetNumber("ts",0.0);
  steeringAdjust = 0.0;
  if (tx > 1.0){
    steeringAdjust = tP * tx - kF;
  }
  else if (tx < -1.0){
    steeringAdjust = tP * tx + kF;
  }
  return(steeringAdjust);
}
void Robot::EncoderVelocity(){
  rightVelocity = m_encoderright.GetVelocity();
  leftVelocity = m_encoderleft.GetVelocity();
  frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoderright.GetVelocity());
  frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoderleft.GetVelocity());
}

//Trajectory
frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          ks, kv, ka),
      kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(kMaxSpeed,
                               kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(kRamseteB,
                             kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          ks, kv, ka),
      kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, 0, 0),
      frc2::PIDController(kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}

//color sensor
void Robot::ColorSensor(){
  

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
