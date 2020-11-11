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
#include "climb.h"

void Robot::RobotInit()
{

  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("camMode", 0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ledMode", 0);
  table->PutNumber("pipeline", pipeline);

  cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  camera.SetResolution(120, 90);
  camera.SetFPS(10);

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
  
  tsrx1.ConfigFactoryDefault();
  vspx1.ConfigFactoryDefault();
  vspx2.ConfigFactoryDefault();

  shift = new frc::DoubleSolenoid(5, 2);
  boost = new frc::DoubleSolenoid(6, 1);
  intake = new frc::DoubleSolenoid(4, 3);
  ahrs = new AHRS(frc::SPI::Port::kMXP);

  //test motor
  m_testMotor.RestoreFactoryDefaults();
  m_shooterMotor.Follow(m_testMotor, true);
  m_testMotor.SetInverted(true);

  //climber
  climberLeftInput = climberLeftSwitch.Get();
  climberRightInput = climberRightSwitch.Get();

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
  kF = 0.15;

  pipeline = 5;

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
  m_encoderright.SetPosition(0);
  m_encoderleft.SetPosition(0);
  ahrs->Reset();
  boost->Set(frc::DoubleSolenoid::kReverse);
  shift->Set(frc::DoubleSolenoid::kForward);
  table->PutNumber("pipeline", pipeline);
  driveForward = false;
  driveForward2 = false;
  driveBackward = false;
  turnLeft = false;
  turnRight = false;
  targetAuto = true;
  targetAuto2 = false;
  autoShoot = false;
  autoShoot2 = false;
  recenterbot = false;
  autointakedown = false;
  shiftGear = false;
  shiftuponce = true;
}
void Robot::AutonomousPeriodic()
{
  autoroutine2();
}
void Robot::TeleopInit()
{
  revs = 0;
  m_colorwheel.Set(0);
  m_encoderleft.SetPosition(0);
  m_encoderright.SetPosition(0);
  table->PutNumber("pipeline", 0);
  m_robotDrive.ArcadeDrive(0, 0);
  boost->Set(frc::DoubleSolenoid::kReverse);
  ahrs->Reset();
  climbmode = false;
}

void Robot::TeleopPeriodic()
{
  getInput();
  double gyroAngle = ahrs->GetAngle();
  frc::SmartDashboard::PutNumber("anglething", gyroAngle);
  double ts = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0);
  frc::SmartDashboard::PutNumber("ts", ts);
  distright = m_encoderright.GetPosition();
  distleft = m_encoderleft.GetPosition();
  frc::SmartDashboard::PutNumber("distright", distright);
  frc::SmartDashboard::PutNumber("distleft", distleft);

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

  index3volt = index2.GetVoltage();
  if (index3volt < 1.3)
  {
    thirdInput = false;
  }
  if (index3volt > 1.7)
  {
    thirdInput = true;
  }
  frc::SmartDashboard::PutBoolean("Ball", thirdInput);

  bool holdup = false;
  if (shiftbuttonpressed == true && shiftup == true)
  {
    shiftup = false;
    holdup = true;
  }
  if (shiftbuttonpressed == true && shiftup == false && holdup == false)
  {
    shiftup = true;
  }
  if (shiftbuttonpressed == false)
  {
    if (shiftup == true)
    {
      //second gear
      shift->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if (shiftup == false)
    {
      //first gear
      shift->Set(frc::DoubleSolenoid::Value::kForward);
    }
  }
  bool waitaminute = false;
  if (boostbuttonpressed == true && boostup == true)
  {
    boostup = false;
    waitaminute = true;
  }
  if (boostbuttonpressed == true && boostup == false && waitaminute == false)
  {
    boostup = true;
  }
  if (boostbuttonpressed == false)
  {
    if (boostup == true)
    {
      boost->Set(frc::DoubleSolenoid::Value::kForward);
    }
    if (boostup == false)
    {
      boost->Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }
  //driver climb control
  bool climbwait = false;
  if (enableclimb == true && climbmode == true)
  {
    climbmode = false;
    climbwait = true;
  }
  if (enableclimb == true && climbmode == false && climbwait == false)
  {
    climbmode = true;
  }
  if (enableclimb == false)
  {
    if (climbmode == true)
    {
      // tsrx1.Set(ControlMode::PercentOutput, leftclimbinput);
      // tsrx2.Set(ControlMode::PercentOutput, rightclimbinput);
      climber(&driver);
    }
    if (climbmode == false)
    {
      // tsrx1.Set(ControlMode::PercentOutput, 0);
      // tsrx2.Set(ControlMode::PercentOutput, 0);
    }
  }
  frc::SmartDashboard::PutBoolean("climbmode", climbmode);

  bool dontcare = false;
  if (intakebuttonpressed == true && intakeup == true)
  {
    intakeup = false;
    dontcare = true;
  }
  if (intakebuttonpressed == true && intakeup == false && dontcare == false)
  {
    intakeup = true;
  }
  if (intakebuttonpressed == false)
  {
    if (intakeup == true)
    {
      intake->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if (intakeup == false)
    {
      intake->Set(frc::DoubleSolenoid::Value::kForward);
    }
  }

  //Operator controls

  //delivery system forward and reverse
  bool waitindex = false;
  if (indextoggle == true && indexauto == true)
  {
    indexauto = false;
    waitindex = true;
  }
  if (indextoggle == true && indexauto == false && waitindex == false)
  {
    indexauto = true;
  }
  if (indextoggle == false)
  {
    if (indexauto == true)
    {
      indexing(false);
    }
    if (indexauto == false)
    {
      if (indexbutton)
      {
        vspx1.Set(ControlMode::PercentOutput, -.8);
        tsrx1.Set(ControlMode::PercentOutput, .8);
      }
      else if (reverseindexbutton)
      {
        vspx1.Set(ControlMode::PercentOutput, .8);
        tsrx1.Set(ControlMode::PercentOutput, -.8);
      }
      else
      {
        vspx1.Set(ControlMode::PercentOutput, 0);
        tsrx1.Set(ControlMode::PercentOutput, 0);
      }
    }
  }
  frc::SmartDashboard::PutBoolean("Auto Index", indexauto);

  testPIDcontroller(&operater, &m_testMotor, false, targetDist);
  //buttons for flywheel
  // if (shooterThrottle)
  // {
  //   //use motor speed from limelight to shoot
  //   table->PutNumber("pipeline", pipeline);
  //   testPIDcontroller(&operater, &m_testMotor, false, targetDist);
  // }
  // else
  // {
  //   //stop motors
  //   m_testMotor.Set(-.1);
  // }
  frc::SmartDashboard::PutNumber("current rpm", m_testEncoder.GetVelocity());
  //left stick axis to spin intake (down = ball in, up = ball out, nothing = no motors)
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
    table->PutNumber("pipeline", pipeline);
    tCorrection = AutoTargetTurn();
    printf("tx %f \n", tx);
  }

  //Turns off Targeting
  if (!autoAlign)
  {
    //table->PutNumber("pipeline", 0);
    tCorrection = 0;
  }
  //Makes the throttle button half the output
  //printf("%f \n", povsetpoints);
  turnMultiplier = .65;
  driveMultiplier = .8;
  m_robotDrive.ArcadeDrive(speed * throttleMultiplier * driveMultiplier, (-rotation * throttleMultiplier * turnMultiplier) + tCorrection); //Drive function
}
void Robot::TestPeriodic()
{
}

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

  //climb controls Back button must be pressed for joystick to work
  enableclimb = driver.GetBackButtonPressed();
  leftclimbinput = driver.GetY(frc::GenericHID::JoystickHand::kLeftHand);
  rightclimbinput = driver.GetY(frc::GenericHID::JoystickHand::kRightHand);

  if (abs(leftclimbinput) < .15)
  {
    leftclimbinput = 0;
  }
  if (abs(rightclimbinput) < .15)
  {
    rightclimbinput = 0;
  }

  povsetpoints = operater.GetPOV();

  shiftbuttonpressed = driver.GetAButtonPressed(); //shifts up and down
  boostbuttonpressed = driver.GetBButtonPressed(); //boosts front up and down

  //autoAlign = m_driveStick.GetRawButton(4);
  //nullTarget = m_driveStick.GetRawButton(5);

  autoAlign = driver.GetXButton();

  //test motor
  startbtn = operater.GetStartButton();

  //left bumper for operator
  shooterThrottle = operater.GetRawButton(5);

  //spits out balls if stuck
  shooterspit = operater.GetPOV();

  //intake down
  intakebuttonpressed = operater.GetAButtonPressed();

  //right bumper
  indexbutton = operater.GetBumper(frc::GenericHID::JoystickHand::kRightHand);

  //b button
  reverseindexbutton = operater.GetBButton();

  indextoggle = operater.GetYButtonPressed();

  //take in ball (axis of left joystick)
  intakeBall = operater.GetY(frc::GenericHID::JoystickHand::kLeftHand);

  //Color Sensor
  //position = operater.GetAButton();   //right bumper
  //rotationControl = operater.GetBButton();   //left bumper
}

double Robot::AutoTargetTurn()
{
  tx = table->GetNumber("tx", 0.0);
  ty = table->GetNumber("ty", 0.0);
  targetDist = (5.6666666 / tan((30.379 + ty) * (wpi::math::pi / 180)));
  frc::SmartDashboard::PutNumber("Distance to Target", targetDist);
  frc::SmartDashboard::PutNumber("tx", tx);
  frc::SmartDashboard::PutNumber("ty", ty);
  double ta = table->GetNumber("ta", 0.0);
  double ts = table->GetNumber("ts", 0.0);
  steeringAdjust = 0.0;
  if (tx > 0.5)
  {
    steeringAdjust = tP * tx - kF;
  }
  else if (tx < -0.5)
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

  if (feet - abs(m_encoderleft.GetPosition()) > -.1)
  {
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
    m_robotDrive.ArcadeDrive(speed, forwardAdjust);
  }
}
void Robot::backwardDrive(double feet, double speed)
{
  double initbackwardangle = ahrs->GetAngle();

  if (feet - abs(m_encoderleft.GetPosition()) < .1)
  {
    double backwardangle = ahrs->GetAngle();
    double angleerror = initbackwardangle - backwardangle;
    double backwardAdjust = 0;
    frc::SmartDashboard::PutNumber("distright", m_encoderright.GetPosition());
    frc::SmartDashboard::PutNumber("distleft", m_encoderleft.GetPosition());

    if (angleerror > 1.0)
    {
      backwardAdjust = -0.025 * angleerror;
    }
    else if (angleerror < -1.0)
    {
      backwardAdjust = 0.025 * angleerror;
    }
    m_robotDrive.ArcadeDrive(-speed, backwardAdjust);
  }
}
void Robot::cwTurn(double turnangleleft)
{
  bool exitleftturn = true;
  double initleftangle;
  double goalleftangle;
  double currentleftangle;
  if (exitleftturn)
  {
    initleftangle = ahrs->GetAngle();
    goalleftangle = initleftangle - turnangleleft;
    currentleftangle = initleftangle;
    exitleftturn = false;
  }
  while (currentleftangle > goalleftangle)
  {
    currentleftangle = ahrs->GetAngle();
    m_robotDrive.ArcadeDrive(0, -.4);
  }
  if (currentleftangle <= goalleftangle)
  {
    return;
  }
}
void Robot::ccwTurn(double turnangleright)
{
  bool exitrightturn = true;
  double initrightangle;
  double goalrightangle;
  double currentrightangle;
  if (exitrightturn)
  {
    initrightangle = ahrs->GetAngle();
    goalrightangle = initrightangle + turnangleright;
    currentrightangle = initrightangle;
    exitrightturn = false;
  }
  while (currentrightangle < goalrightangle)
  {
    currentrightangle = ahrs->GetAngle();
    m_robotDrive.ArcadeDrive(0, .4);
  }
  if (currentrightangle >= goalrightangle)
  {
    return;
  }
}
void Robot::brakeRobot()
{
  m_robotDrive.ArcadeDrive(0, 0);
}

void Robot::autoroutine1()
{
  gyroAngle = ahrs->GetAngle();
  frc::SmartDashboard::PutNumber("anglething", gyroAngle);
  indexing(true);
  if (targetAuto)
  {
    tCorrection = AutoTargetTurn();
    m_robotDrive.ArcadeDrive(0, tCorrection);
    printf("tCorrection %f \n", tCorrection);
    printf("targeting \n");
    if (abs(tx) < 0.5 && tx != 0)
    {
      targetAuto = false;
      autoShoot = true;
    }
  }
  if (autoShoot)
  {
    if (autocounter < 175)
    {
      autocounter++;
      //printf("counter %d \n", autocounter);
      testPIDcontroller(&operater, &m_testMotor, true, targetDist);
      if (autocounter > 50)
      {
        vspx2.Set(ControlMode::PercentOutput, .7);
      }
    }
    if (autocounter == 175)
    {
      testPIDcontroller(&operater, &m_testMotor, false, targetDist);
      vspx2.Set(ControlMode::PercentOutput, 0);
      autocounter = 0;
      autoShoot = false;
      recenterbot = true;
    }
  }
  if (recenterbot)
  {
    if (gyroAngle < -1.0)
    {
      m_robotDrive.ArcadeDrive(0, .4);
    }
    else if (gyroAngle > 1.0)
    {
      m_robotDrive.ArcadeDrive(0, -.4);
    }
    else
    {
      m_robotDrive.ArcadeDrive(0, 0);
      recenterbot = false;
      driveForward = true;
    }
  }
  // if(turnLeft){
  //   ccwTurn(40);
  //   turnLeft = false;
  //   driveForward = true;
  // }
  if (driveForward)
  {
    forwardDrive(9.72222, 0.7);
    if (shiftuponce)
    {
      shift->Set(frc::DoubleSolenoid::kReverse);
      shiftuponce = false;
    }
    if (9.72222 - abs(m_encoderleft.GetPosition()) <= .1)
    {
      driveForward = false;
      driveBackward = true;
      shift->Set(frc::DoubleSolenoid::kReverse);
    }
  }
  if (2.72222 - abs(m_encoderleft.GetPosition()) <= .1 && driveForward)
  {
    intake->Set(frc::DoubleSolenoid::kForward);
    shift->Set(frc::DoubleSolenoid::kForward);
    vspx2.Set(ControlMode::PercentOutput, -.75);
  }
  if (driveBackward)
  {
    shift->Set(frc::DoubleSolenoid::kReverse);
    backwardDrive(8.83333, 0.7);
    if (7.72222 - abs(m_encoderleft.GetPosition()) >= -.1)
    {
      driveBackward = false;
      targetAuto2 = true;
    }
  }
  if (targetAuto2)
  {
    tCorrection = AutoTargetTurn();
    m_robotDrive.ArcadeDrive(0, tCorrection);
    printf("tCorrection %f \n", tCorrection);
    printf("targeting \n");
    if (abs(tx) < 0.5 && tx != 0)
    {
      targetAuto2 = false;
      autoShoot2 = true;
    }
  }
  if (autoShoot2)
  {
    if (autocounter < 175)
    {
      autocounter++;
      printf("counter %d \n", autocounter);
      testPIDcontroller(&operater, &m_testMotor, true, targetDist);
      if (autocounter > 50)
      {
        vspx2.Set(ControlMode::PercentOutput, .7);
      }
    }
    if (autocounter == 175)
    {
      testPIDcontroller(&operater, &m_testMotor, false, targetDist);
      vspx2.Set(ControlMode::PercentOutput, 0);
      autoShoot2 = false;
    }
  }
}

void Robot::autoroutine2()
{
  indexing(true);
  if (targetAuto)
  {
    tCorrection = AutoTargetTurn();
    m_robotDrive.ArcadeDrive(0, tCorrection);
    printf("tCorrection %f \n", tCorrection);
    printf("targeting \n");
    if (abs(tx) < 0.5 && tx != 0)
    {
      targetAuto = false;
      autoShoot = true;
    }
  }
  if (autoShoot)
  {
    if (autocounter < 175)
    {
      autocounter++;
      //printf("counter %d \n", autocounter);
      testPIDcontroller(&operater, &m_testMotor, true, targetDist);
      if (autocounter > 50)
      {
        vspx2.Set(ControlMode::PercentOutput, .7);
      }
    }
    if (autocounter == 175)
    {
      testPIDcontroller(&operater, &m_testMotor, false, targetDist);
      vspx2.Set(ControlMode::PercentOutput, 0);
      autocounter = 0;
      autoShoot = false;
      recenterbot = true;
    }
  }
  if (recenterbot)
  {
    if (gyroAngle < -1.0)
    {
      m_robotDrive.ArcadeDrive(0, .4);
    }
    else if (gyroAngle > 1.0)
    {
      m_robotDrive.ArcadeDrive(0, -.4);
    }
    else
    {
      m_robotDrive.ArcadeDrive(0, 0);
      recenterbot = false;
      driveForward = true;
    }
  }
  if (driveForward)
  {
    forwardDrive(3.5, 0.7);
    if (3.5 - abs(m_encoderleft.GetPosition()) <= .1)
    {
      driveForward = false;
    }
  }
}
void Robot::autoroutine3()
{
  indexing(true);
  if (targetAuto)
  {
    tCorrection = AutoTargetTurn();
    m_robotDrive.ArcadeDrive(0, tCorrection);
    printf("tCorrection %f \n", tCorrection);
    printf("targeting \n");
    if (abs(tx) < 0.5 && tx != 0)
    {
      targetAuto = false;
      autoShoot = true;
    }
  }
  if (autoShoot)
  {
    if (autocounter < 175) 
    {
      autocounter++;
      //printf("counter %d \n", autocounter);
      testPIDcontroller(&operater, &m_testMotor, true, targetDist);
      if (autocounter > 50)
      {
        vspx2.Set(ControlMode::PercentOutput, .7);
      }
    }
    if (autocounter == 175)
    {
      testPIDcontroller(&operater, &m_testMotor, false, targetDist);
      vspx2.Set(ControlMode::PercentOutput, 0);
      autocounter = 0;
      autoShoot = false;
      recenterbot = true;
    }
  }
  if (recenterbot)
  {
    if (gyroAngle < -1.0)
    {
      m_robotDrive.ArcadeDrive(0, .4);
    }
    else if (gyroAngle > 1.0)
    {
      m_robotDrive.ArcadeDrive(0, -.4);
    }
    else
    {
      m_robotDrive.ArcadeDrive(0, 0);
      recenterbot = false;
      driveBackward = true;
    }
  }
  if (driveBackward)
  {
    backwardDrive(-4, 0.7);
    if (6 - abs(m_encoderleft.GetPosition()) >= -.1)
    {
      driveBackward = false;
    }
  }
}
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif



// thousandth line of code