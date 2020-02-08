/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit()
{
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

  // set shooter PID coefficients
  m_shooterPIDcontroller.SetP(shooterkP);
  m_shooterPIDcontroller.SetI(shooterkI);
  m_shooterPIDcontroller.SetD(shooterkD);
  m_shooterPIDcontroller.SetIZone(shooterkIz);
  m_shooterPIDcontroller.SetFF(shooterkFF);
  m_shooterPIDcontroller.SetOutputRange(shooterkMinOutput, shooterkMaxOutput);

  // display shooter PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("shooter P Gain", shooterkP);
  frc::SmartDashboard::PutNumber("shooter I Gain", shooterkI);
  frc::SmartDashboard::PutNumber("shooter D Gain", shooterkD);
  frc::SmartDashboard::PutNumber("shooter I Zone", shooterkIz);
  frc::SmartDashboard::PutNumber("shooter Feed Forward", shooterkFF);
  frc::SmartDashboard::PutNumber("shooter Max Output", shooterkMaxOutput);
  frc::SmartDashboard::PutNumber("shooter Min Output", shooterkMinOutput);

  //drive motors
  m_encoderleft.SetPositionConversionFactor((wpi::math::pi) / (7 * 3));
  m_encoderright.SetPositionConversionFactor((wpi::math::pi) / (7 * 3));
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
  kF = frc::SmartDashboard::GetNumber("fP", 0.05);

  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0);
  frc::SmartDashboard::PutNumber("ts", ts);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);

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
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
  table->PutNumber("pipeline", 4);
}
void Robot::AutonomousPeriodic()
{
  m_testMotor.Set(.1);
  double gyroAngle = ahrs->GetAngle();
  frc::SmartDashboard::PutNumber("anglething", gyroAngle);

  bool inputVal = testInput.Get();
  frc::SmartDashboard::PutString("inputVal", "False");
  if (!inputVal)
  {
    frc::SmartDashboard::PutString("inputVal", "False");
  }
  if (inputVal)
  {
    frc::SmartDashboard::PutString("inputVal", "True");
  }
  else
  {
    frc::SmartDashboard::PutString("inputVal", "big gay");
  }
}

void Robot::TeleopInit()
{
  shift->Set(frc::DoubleSolenoid::Value::kReverse);
  revs = 0;
  m_colorwheel.Set(0);
}

void Robot::TeleopPeriodic()
{
  getInput();
  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0);
  frc::SmartDashboard::PutNumber("ts", ts);

  readColorSensor();
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
  }

  if (shiftUp)
  {
    shift->Set(frc::DoubleSolenoid::Value::kReverse);
    bool gear = false;
  }
  if (shiftDown)
  {
    shift->Set(frc::DoubleSolenoid::Value::kForward);
    bool gear = true;
  }
  testPIDcontroller();
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
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 0);
    table->PutNumber("pipeline", 1);
    tCorrection = AutoTargetTurn();
  }
  //Turns off Targeting
  else
  {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 1);
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

  //shiftDown = m_driveStick.GetRawButton(2); //shift up and down buttons
  //shiftUp = m_driveStick.GetRawButton(3);

  shiftDown = driver.GetBButton();
  shiftUp = driver.GetAButton();

  //autoAlign = m_driveStick.GetRawButton(4);
  //nullTarget = m_driveStick.GetRawButton(5);

  autoAlign = driver.GetStartButton();
  nullTarget = driver.GetRawButton(10);

  //test motor
  testABtn = operater.GetAButton();
  testBBtn = operater.GetBButton();
  testXBtn = operater.GetXButton();
  testYBtn = operater.GetYButton();
  RPMstick = operater.GetY(frc::GenericHID::JoystickHand::kLeftHand);
  startbtn = operater.GetStartButton();

  //Color Sensor
  //position = operater.GetAButton();   //right bumper
  //rotationControl = operater.GetBButton();   //left bumper
}

double Robot::AutoTargetTurn()
{
  double tx = table->GetNumber("tx", 0.0);
  ty = table->GetNumber("ty", 0.0);
  targetDist = (5.5 / tan((30 + ty) * (wpi::math::pi / 180)));
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

//color sensor
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
          Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      case 'G':
        if (currentColor == "Yellow")
        {
          Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      case 'R':
        if (currentColor == "Blue")
        {
          Wait(.25);
          colorPIDcontroller(0);
          initp = false;
        }
        break;
      case 'Y':
        if (currentColor == "Green")
        {
          Wait(.25);
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
void Robot::readColorSensor()
{
  frc::Color detectedColor = m_colorSensor.GetColor();
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
  //gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  gameData = "Y";
  if (matchedColor == kBlueTarget && confidence >= .95)
  {
    currentColor = "Blue";
  }
  else if (matchedColor == kRedTarget && confidence >= .95)
  {
    currentColor = "Red";
  }
  else if (matchedColor == kGreenTarget && confidence >= .98)
  {
    currentColor = "Green";
  }
  else if (matchedColor == kYellowTarget && confidence >= .95)
  {
    currentColor = "Yellow";
  }
  else
  {
    currentColor = "Unknown";
  }

  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Color", currentColor);
  frc::SmartDashboard::PutString("target to", gameData);
}

void Robot::testPIDcontroller()
{
  // read PID coefficients from SmartDashboard
  double testp = frc::SmartDashboard::GetNumber("test P Gain", 0.000250);
  double testi = frc::SmartDashboard::GetNumber("test I Gain", 0);
  double testd = frc::SmartDashboard::GetNumber("test D Gain", .000800);
  double testiz = frc::SmartDashboard::GetNumber("test I Zone", 0);
  double testff = frc::SmartDashboard::GetNumber("test Feed Forward", 0.000015);
  double testmax = frc::SmartDashboard::GetNumber("test Max Output", 0.85);
  double testmin = frc::SmartDashboard::GetNumber("test Min Output", -0.85);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if ((testp != testkP))
  {
    m_testpidController.SetP(testp);
    testkP = testp;
  }
  if ((testi != testkI))
  {
    m_testpidController.SetI(testi * 1e-6);
    testkI = testi;
  }
  if ((testd != testkD))
  {
    m_testpidController.SetD(testd);
    testkD = testd;
  }
  //if((testiz != testkIz)) { m_testpidController.SetIZone(testiz); testkIz = testiz; }
  if ((testff != testkFF))
  {
    m_testpidController.SetFF(testff);
    testkFF = testff;
  }
  if ((testmax != testkMaxOutput) || (testmin != testkMinOutput))
  {
    m_testpidController.SetOutputRange(testmin, testmax);
    testkMinOutput = testmin;
    testkMaxOutput = testmax;
  }

  if (testABtn)
  {
    testSetPoint = 1000;
  }
  else if (testBBtn)
  {
    testSetPoint = 2000;
  }
  else if (testYBtn)
  {
    testSetPoint = 3000;
  }
  else if (testXBtn)
  {
    testSetPoint = 4000;
  }
  else if (-.1 > RPMstick || .1 < RPMstick)
  {
    testSetPoint = testMaxRPM * RPMstick;
  }
  else
  {
    m_testMotor.Set(0);
  }
  if (-.1 > RPMstick || .1 < RPMstick || testXBtn || testYBtn || testBBtn || testABtn)
  {
    if (testSetPoint >= 1000)
    {
      testiz = testSetPoint - 1000;
    }
    else
    {
      testiz = 0;
    }
    m_testpidController.SetIZone(testiz);
    m_testpidController.SetReference(testSetPoint, rev::ControlType::kVelocity);
  }
  frc::SmartDashboard::PutNumber("Motor temps", m_testMotor.GetMotorTemperature());
  frc::SmartDashboard::PutNumber("Motor temps", m_shooterMotor.GetMotorTemperature());
  frc::SmartDashboard::PutNumber("SetPoint", testSetPoint * 1.33333);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_testEncoder.GetVelocity() * 1.333333);
}
void Robot::colorPIDcontroller(double colorSetPoint)
{
  // read PID coefficients from SmartDashboard
  double colorp = frc::SmartDashboard::GetNumber("color P Gain", 0);
  double colori = frc::SmartDashboard::GetNumber("color I Gain", 0);
  double colord = frc::SmartDashboard::GetNumber("color D Gain", 0);
  double coloriz = frc::SmartDashboard::GetNumber("color I Zone", 0);
  double colorff = frc::SmartDashboard::GetNumber("color Feed Forward", 0);
  double colormax = frc::SmartDashboard::GetNumber("color Max Output", 0);
  double colormin = frc::SmartDashboard::GetNumber("color Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if ((colorp != colorkP))
  {
    m_colorPIDcontroller.SetP(colorp);
    colorkP = colorp;
  }
  if ((colori != colorkI))
  {
    m_colorPIDcontroller.SetI(colori);
    colorkI = colori;
  }
  if ((colord != colorkD))
  {
    m_colorPIDcontroller.SetD(colord);
    colorkD = colord;
  }
  if ((coloriz != colorkIz))
  {
    m_colorPIDcontroller.SetIZone(coloriz);
    colorkIz = coloriz;
  }
  if ((colorff != colorkFF))
  {
    m_colorPIDcontroller.SetFF(colorff);
    colorkFF = colorff;
  }
  if ((colormax != colorkMaxOutput) || (colormin != colorkMinOutput))
  {
    colorkMinOutput = colormin;
    colorkMaxOutput = colormax;
  }
  m_colorPIDcontroller.SetOutputRange(colormin, colormax);
  m_colorPIDcontroller.SetReference(colorSetPoint, rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("SetPoint", colorSetPoint);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_colorencoder.GetVelocity());
}
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

void Robot::shooterPIDcontroller(double shooterSetPoint)
{
  // read PID coefficients from SmartDashboard
  double shooterp = frc::SmartDashboard::GetNumber("shooter P Gain", 0);
  double shooteri = frc::SmartDashboard::GetNumber("shooter I Gain", 0);
  double shooterd = frc::SmartDashboard::GetNumber("shooter D Gain", 0);
  double shooteriz = frc::SmartDashboard::GetNumber("shooter I Zone", 0);
  double shooterff = frc::SmartDashboard::GetNumber("shooter Feed Forward", 0);
  double shootermax = frc::SmartDashboard::GetNumber("shooter Max Output", 0);
  double shootermin = frc::SmartDashboard::GetNumber("shooter Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if ((shooterp != shooterkP))
  {
    m_shooterPIDcontroller.SetP(shooterp);
    shooterkP = shooterp;
  }
  if ((shooteri != shooterkI))
  {
    m_shooterPIDcontroller.SetI(shooteri);
    shooterkI = shooteri;
  }
  if ((shooterd != shooterkD))
  {
    m_shooterPIDcontroller.SetD(shooterd);
    shooterkD = shooterd;
  }
  if ((shooteriz != shooterkIz))
  {
    m_shooterPIDcontroller.SetIZone(shooteriz);
    shooterkIz = shooteriz;
  }
  if ((shooterff != shooterkFF))
  {
    m_shooterPIDcontroller.SetFF(shooterff);
    shooterkFF = shooterff;
  }
  if ((shootermax != shooterkMaxOutput) || (shootermin != shooterkMinOutput))
  {
    shooterkMinOutput = shootermin;
    shooterkMaxOutput = shootermax;
  }
  m_shooterPIDcontroller.SetOutputRange(shootermin, shootermax);
  m_shooterPIDcontroller.SetReference(shooterSetPoint, rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("SetPoint", shooterSetPoint);
  frc::SmartDashboard::PutNumber("ProcessVariable", m_shooterencoder.GetVelocity());
}

void Robot::forwardDrive(double feet, double speed)
{
  double leftRPM = (speed * 420) / (wpi::math::pi / 3);
  double rightRPM = (speed * 420) / (wpi::math::pi / 3);
  while (m_encoderright.GetPosition() < feet || m_encoderleft.GetPosition() < feet)
  {
    rightPIDcontroller(rightRPM);
    leftPIDcontroller(leftRPM);
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