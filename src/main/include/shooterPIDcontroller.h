#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

//test motor
rev::CANSparkMax m_testMotor{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANPIDController m_testpidController = m_testMotor.GetPIDController();
rev::CANEncoder m_testEncoder = m_testMotor.GetEncoder();
double testkP = 0.000300, testkI = 1, testkD = 0, testkIz = 0, testkFF = 0.000040, testkMaxOutput = .85, testkMinOutput = -.85;



void testPIDcontroller(frc::XboxController * shooteroperater,  bool autoShoot)
{
    //test motor PID
    double testMaxRPM = 5700;
    double testSetPoint;
    double RPMstick;
    double povsetpoint;
    //test motor
    povsetpoint = shooteroperater->GetPOV();
    RPMstick = shooteroperater->GetY(frc::GenericHID::JoystickHand::kLeftHand);
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
    if (povsetpoint == 0)
    {
        testSetPoint = 2700;
    }
    else if (povsetpoint == 45 || autoShoot)
    {
        testSetPoint = 2800;
    }    
    else if (povsetpoint == 90)
    {
        testSetPoint = 2900;
    }
    else if (povsetpoint == 135)
    {
        testSetPoint = 3000;
    }
    else if (povsetpoint == 180)
    {
        testSetPoint = 3100;
    }
    else if (povsetpoint == 225)
    {
        testSetPoint = 3200;
    }
    else if (povsetpoint == 270)
    {
        testSetPoint = 3300;
    }
    else if (povsetpoint == 315)
    {
        testSetPoint = 3400;
    }
    else if (povsetpoint == -1 && autoShoot == false){
        m_testMotor.Set(0);
    }
    else if (-.1 >= RPMstick || .1 <= RPMstick)
    {
        testSetPoint = testMaxRPM * RPMstick;
    }
    else
    {
        m_testMotor.Set(0);
    }
    if (RPMstick <= -.1 || .1 <= RPMstick || povsetpoint == 0 || autoShoot
        || povsetpoint == 90 || povsetpoint == 180 || povsetpoint == 270)
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
        m_testpidController.SetReference(-testSetPoint, rev::ControlType::kVelocity);
    }
    frc::SmartDashboard::PutNumber("Motor temps", m_testMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("SetPoint", testSetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_testEncoder.GetVelocity());
}