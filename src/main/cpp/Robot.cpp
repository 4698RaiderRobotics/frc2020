/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#define DEBUG_SWITCH1
//#define DEBUG_SWITCH2
//#define DEBUG_SWITCH3
#include "Robot.h"

void Robot::RobotInit()
{
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("camMode",0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ledMode",0);

  cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  camera.SetResolution(160,120);

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

  vspx.ConfigFactoryDefault();
  tsrx1.ConfigFactoryDefault();
  tsrx2.ConfigFactoryDefault();

  tsrx2.Follow(tsrx1);

  shift = new frc::DoubleSolenoid(5 , 2);
  boost = new frc::DoubleSolenoid(6 , 1);
  intake = new frc::DoubleSolenoid(4 , 3);
  ahrs = new AHRS(frc::SPI::Port::kMXP);

  //test motor
  m_testMotor.RestoreFactoryDefaults();
  m_shooterMotor.Follow(m_testMotor, true);
  m_testMotor.SetInverted(true);

  // set test PID coefficients
  m_testpidController.SetP(testkP);
  m_testpidController.SetI(testkI * 1e-6);
  m_testpidController.SetD(testkD);
  m_testpidController.SetIZone(testkIz);
  m_testpidController.SetFF(testkFF);
  m_testpidController.SetOutputRange(testkMinOutput, testkMaxOutput);

  // display test PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("test P Gain", testkP);
  frc::SmartDashboard::PutNumber("test I Gain", testkI);
  frc::SmartDashboard::PutNumber("test D Gain", testkD);
  frc::SmartDashboard::PutNumber("test I Zone", testkIz);
  frc::SmartDashboard::PutNumber("test Feed Forward", testkFF);
  frc::SmartDashboard::PutNumber("test Max Output", testkMaxOutput);
  frc::SmartDashboard::PutNumber("test Min Output", testkMinOutput);

  // set color PID coefficients
  m_colorPIDcontroller.SetP(colorkP);
  m_colorPIDcontroller.SetI(colorkI);
  m_colorPIDcontroller.SetD(colorkD);
  m_colorPIDcontroller.SetIZone(colorkIz);
  m_colorPIDcontroller.SetFF(colorkFF);
  m_colorPIDcontroller.SetOutputRange(colorkMinOutput, colorkMaxOutput);

  // display color PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("color P Gain", colorkP);
  frc::SmartDashboard::PutNumber("color I Gain", colorkI);
  frc::SmartDashboard::PutNumber("color D Gain", colorkD);
  frc::SmartDashboard::PutNumber("color I Zone", colorkIz);
  frc::SmartDashboard::PutNumber("color Feed Forward", colorkFF);
  frc::SmartDashboard::PutNumber("color Max Output", colorkMaxOutput);
  frc::SmartDashboard::PutNumber("color Min Output", colorkMinOutput);

  //Color sensor
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);

  //Color sensor Motor and Encoder
  m_colorencoder.SetPositionConversionFactor(1 / 16);
  m_colorencoder.SetPosition(0);

  //drive motors
  m_encoderleft.SetPositionConversionFactor((4 * wpi::math::pi) / (18 * 12));
  m_encoderright.SetPositionConversionFactor((4 * wpi::math::pi) / (18 * 12));
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  tP = frc::SmartDashboard::GetNumber("tP", -0.025);
  kF = 0.1;

  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0);
  frc::SmartDashboard::PutNumber("ts", ts);

  // read PID coefficients from SmartDashboard
  double drivep = frc::SmartDashboard::GetNumber("drive P Gain", 0);
  double drivei = frc::SmartDashboard::GetNumber("drive I Gain", 0);
  double drived = frc::SmartDashboard::GetNumber("drive D Gain", 0);
  double driveiz = frc::SmartDashboard::GetNumber("drive I Zone", 0);
  double driveff = frc::SmartDashboard::GetNumber("drive Feed Forward", 0);
  double drivemax = frc::SmartDashboard::GetNumber("drive Max Output", 0);
  double drivemin = frc::SmartDashboard::GetNumber("drive Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if ((drivep != leftkP))
  {
    m_leftPIDcontroller.SetP(drivep);
    m_rightPIDcontroller.SetP(drivep);
    leftkP = drivep;
    rightkP = drivep;
  }
  if ((drivei != leftkI))
  {
    m_leftPIDcontroller.SetP(drivei);
    m_rightPIDcontroller.SetP(drivei);
    leftkI = drivei;
    rightkI = drivei;
  }
  if ((drived != leftkD))
  {
    m_leftPIDcontroller.SetP(drived);
    m_rightPIDcontroller.SetP(drived);
    leftkD = drived;
    rightkD = drived;
  }
  if ((driveiz != leftkIz))
  {
    m_leftPIDcontroller.SetP(driveiz);
    m_rightPIDcontroller.SetP(driveiz);
    leftkIz = driveiz;
    rightkIz = driveiz;
  }
  if ((driveff != leftkFF))
  {
    m_leftPIDcontroller.SetP(driveff);
    m_rightPIDcontroller.SetP(driveff);
    leftkFF = driveff;
    rightkFF = driveff;
  }

  if ((drivemax != leftkMaxOutput) || (drivemin != leftkMinOutput))
  {
    leftkMinOutput = drivemin;
    leftkMaxOutput = drivemax;
    rightkMinOutput = drivemin;
    rightkMaxOutput = drivemax;
  }
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
void Robot::AutonomousInit()
{
  ahrs->Reset();
  m_encoderright.SetPosition(0);
  m_encoderleft.SetPosition(0);
}
void Robot::AutonomousPeriodic()
{
  double gyroAngle = ahrs->GetAngle();
  frc::SmartDashboard::PutNumber("anglething", gyroAngle);
  bool once = true;
  if(once){
    forwardDrive(5, 1);
    once = false;
  }
}
void Robot::TeleopInit()
{
  revs = 0;
  m_colorwheel.Set(0);
  m_encoderleft.SetPosition(0);
  m_encoderright.SetPosition(0);
}

void Robot::TeleopPeriodic()
{
  getInput();
  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0);
  frc::SmartDashboard::PutNumber("ts", ts);
  distright = m_encoderright.GetPosition();
  distleft = m_encoderleft.GetPosition();
  frc::SmartDashboard::PutNumber("distright", distright);
  frc::SmartDashboard::PutNumber("distleft", distleft);
  //indexing();

  /*readColorSensor();
  if (position || rotationControl)
  {
    executeColorSensor();
  }
  if (rotationControl == false && initr == false)
  {
    colorPIDcontroller(0);
    initr = true;
    revs = 0;
  }
  if (position == false && initp == false)
  {
    initp = true;
  }*/


  bool holdup = false;
  if(shiftbuttonpressed == true && shiftup == true){
   shiftup = false;
   holdup = true;
  }
  if(shiftbuttonpressed == true && shiftup == false && holdup == false){
    shiftup = true;
  }
  if (shiftbuttonpressed == false){
    if (shiftup == true){
      shift->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if (shiftup == false){
      shift->Set(frc::DoubleSolenoid::Value::kForward);
    }
  }

  bool waitaminute = false;
  if(boostbuttonpressed == true && boostup == true){
   boostup = false;
   waitaminute = true;
  }
  if(boostbuttonpressed == true && boostup == false && waitaminute == false){
    boostup = true;
  }
  if (boostbuttonpressed == false){
    if (boostup == true){
      boost->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if (boostup == false){
      boost->Set(frc::DoubleSolenoid::Value::kForward);
    }
  }

  bool dontcare = false;
  if(intakebuttonpressed == true && intakeup == true){
   intakeup = false;
   dontcare = true;
  }
  if(intakebuttonpressed == true && intakeup == false && dontcare == false){
    intakeup = true;
  }
  if (intakebuttonpressed == false){
    if (intakeup == true){
      intake->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if (intakeup == false){
      intake->Set(frc::DoubleSolenoid::Value::kForward);
    }
  }

  /*
  bool indexwait = false;
  if(indexbuttonpressed == true && indexup == true){
   indexup = false;
   indexwait = true;
  }
  if(indexbuttonpressed == true && indexup == false && indexwait == false){
    indexup = true;
  }
  if (indexbuttonpressed == false){
    if (indexup == true){
      vspx.Set(ControlMode::PercentOutput, .5);
    }
    if (indexup == false){
      vspx.Set(ControlMode::PercentOutput, 0);
    }
  }
  */

  if(indexbuttonpressed){
    vspx.Set(ControlMode::PercentOutput, .5);
  }
  if(!indexbuttonpressed){
    vspx.Set(ControlMode::PercentOutput, 0);
  }
  if(reverseindexbutton){
    vspx.Set(ControlMode::PercentOutput, -.5);
  }
  if(!reverseindexbutton){
    vspx.Set(ControlMode::PercentOutput, 0);
  }
  //Operator controls

  //buttons for flywheel
  if (shooterThrottle)
  {
    //use motor speed from limelight to shoot
    testPIDcontroller(&operater);
  }
  if (!shooterThrottle)
  {
    //stop motors
    m_testMotor.Set(0);
  }

  //left stick axis to spin intake
  if (intakeBall > 0.5)
  {
    vspx2.Set(ControlMode::PercentOutput, -.75);
  }
  if (intakeBall < 0.5 && intakeBall > -0.5)
  {
    vspx2.Set(ControlMode::PercentOutput, 0);
  }
  if (intakeBall < -0.5)
  {
    vspx2.Set(ControlMode::PercentOutput, .75);
  }

  //testPIDcontroller(&operater);
  
  
  //Sets shifter to high or low gear
  if (throttle)
  {
    throttleMultiplier = .5;
  }
  else
  {
    throttleMultiplier = 1;
  }


  //Limelight Target Reflective Strips
  if (autoAlign)
  {
    table->PutNumber("pipeline", 4);
    tCorrection = AutoTargetTurn();
  }
  //Turns off Targeting
  else
  {
    tCorrection = 0;
  }
  //Makes the throttle button half the output
  turnMultiplier = .6;
  driveMultiplier = .8;
  m_robotDrive.ArcadeDrive(speed * throttleMultiplier * driveMultiplier, (-rotation * throttleMultiplier * turnMultiplier) + tCorrection); //Drive function
}

void Robot::TestPeriodic() {}

void Robot::getInput()
{
  rotation = driver.GetX(frc::GenericHID::JoystickHand::kLeftHand); //setting controller x value to turn value
  //speed = m_driveStick.GetY();
  //rotation = m_driveStick.GetX();
  RT = driver.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand); //setting right trigger to forward
  LT = driver.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);  //setting left trigger to backward
  speed = RT - LT;
  //throttle = m_driveStick.GetRawButton(1);  //used for halving speed
  throttle = driver.GetRawButton(6); //right bumper throttles speed

  shiftbuttonpressed = driver.GetAButtonPressed(); //shifts up and down
  boostbuttonpressed = driver.GetBButtonPressed(); //boosts front up and down

  //autoAlign = m_driveStick.GetRawButton(4);
  //nullTarget = m_driveStick.GetRawButton(5);

  autoAlign = driver.GetStartButton();
  nullTarget = driver.GetXButton();

  //test motor
  startbtn = operater.GetStartButton();

  //left bumper for operator
  shooterThrottle = operater.GetRawButton(5);

  //release ball to be shot (right bumper)
  //moveBall = operater.GetRawButton(6);

  //intake down
  intakebuttonpressed = operater.GetAButtonPressed();
  indexbuttonpressed = operater.GetRawButton(6);
  reverseindexbutton = operater.GetBButtonPressed();
  
  //take in ball
  intakeBall = operater.GetY(frc::GenericHID::JoystickHand::kLeftHand);

  //Color Sensor
  //position = operater.GetAButton();   //right bumper
  //rotationControl = operater.GetBButton();   //left bumper
}

double Robot::AutoTargetTurn()
{
  double tx = table->GetNumber("tx", 0.0);
  ty = table->GetNumber("ty", 0.0);
  targetDist = (5.666666 / tan((49.05 + ty) * (wpi::math::pi / 180)));
  frc::SmartDashboard::PutNumber("Distance to Target", targetDist);
  double ta = table->GetNumber("ta", 0.0);
  double ts = table->GetNumber("ts", 0.0);
  steeringAdjust = 0.0;
  if (tx > 1.0)
  {
    steeringAdjust = tP * tx - kF;
  }
  else if (tx < -1.0)
  {
    steeringAdjust = tP * tx + kF;
  }
  return (steeringAdjust);
}

/*double Robot::rampMotorSpeed(){
  ty = table->GetNumber("ty", 0.0);
  targetDist = (5.5 / tan((30 + ty) * (wpi::math::pi / 180)));
  return(targetDist);
}*/

void Robot::rightPIDcontroller(double rightSetPoint)
{
  m_rightPIDcontroller.SetOutputRange(rightkMinOutput, rightkMaxOutput);
  m_rightPIDcontroller.SetReference(rightSetPoint, rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("SetPoint", rightSetPoint);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_encoderright.GetVelocity());
}
void Robot::leftPIDcontroller(double leftSetPoint)
{
  m_leftPIDcontroller.SetOutputRange(leftkMinOutput, leftkMaxOutput);
  m_leftPIDcontroller.SetReference(leftSetPoint, rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("SetPoint", leftSetPoint);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_encoderleft.GetVelocity());
}

void Robot::executeColorSensor()
{
  if (position && initp)
  {
    readColorSensor();
    colorPIDcontroller(960);

    if (gameData.length() > 0)
    {
      switch (gameData[0])
      {
      case 'B':
        if (currentColor == "Red")
        {
          frc::Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      case 'G':
        if (currentColor == "Yellow")
        {
          frc::Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      case 'R':
        if (currentColor == "Blue")
        {
          frc::Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      case 'Y':
        if (currentColor == "Green")
        {
          frc::Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      default:
        colorPIDcontroller(0);
        break;
      }
    }
  }

  if (rotationControl && revs <= 8)
  {
    readColorSensor();

    if (initr)
    {
      initColor = currentColor;
      lastColor = initColor;
      revs = 0;
      colorPIDcontroller(2500);
      if (initColor == "Unknown")
      {
        initr = true;
      }
      else
      {
        initr = false;
      }
    }
    if (lastColor == initColor && currentColor != initColor)
    {
      revs += 1;
    }
    frc::SmartDashboard::PutNumber("revs", revs);
    frc::SmartDashboard::PutString("initColor", initColor);
    frc::SmartDashboard::PutString("currentColor", currentColor);
    frc::SmartDashboard::PutString("lastColor", lastColor);
    lastColor = currentColor;
    if (revs > 8)
    {
      colorPIDcontroller(0);
    }
  }
}

void Robot::forwardDrive(double feet, double speed)
{
  double initforwardangle = ahrs->GetAngle();

  while(feet - abs(m_encoderleft.GetPosition()) > .1){
    double forwardangle = ahrs->GetAngle();
    double angleerror = initforwardangle - forwardangle;
    double forwardAdjust = 0;
    frc::SmartDashboard::PutNumber("distright", m_encoderright.GetPosition());
    frc::SmartDashboard::PutNumber("distleft", m_encoderleft.GetPosition());

    if (angleerror > 1.0)
    {
      forwardAdjust = -0.025 * angleerror;
    }
    else if (angleerror < -1.0)
    {
      forwardAdjust = 0.025 * angleerror;
    }

    double leftRPM = speed / (((4 * wpi::math::pi) / (12 * 18)) / 60);
    double rightRPM = speed / (((4 * wpi::math::pi) / (12 * 18)) / 60);
    m_robotDrive.ArcadeDrive(.5, forwardAdjust);
  }
}
void Robot::leftTurn(double turnangleleft)
{
  double initleftangle = ahrs->GetAngle();
  double goalleftangle = initleftangle - turnangleleft;
  double currentleftangle = initleftangle;
  while (currentleftangle > goalleftangle)
  {
    currentleftangle = ahrs->GetAngle();
    rightPIDcontroller(960);
    leftPIDcontroller(-960);
  }
  if (currentleftangle <= goalleftangle)
  {
    brakeRobot();
  }
}
void Robot::rightTurn(double turnangleright)
{
  double initrightangle = ahrs->GetAngle();
  double goalrightangle = initrightangle + turnangleright;
  double currentrightangle = initrightangle;
  while (currentrightangle < goalrightangle)
  {
    currentrightangle = ahrs->GetAngle();
    rightPIDcontroller(-960);
    leftPIDcontroller(960);
  }
  if (currentrightangle >= goalrightangle)
  {
    brakeRobot();
  }
}
void Robot::brakeRobot()
{
  rightPIDcontroller(0);
  leftPIDcontroller(0);
}
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif