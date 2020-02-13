#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <wpi/math>

std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 

//test motor
rev::CANSparkMax m_testMotor{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANPIDController m_testpidController = m_testMotor.GetPIDController();
rev::CANEncoder m_testEncoder = m_testMotor.GetEncoder();
double testkP = 0.000300, testkI = 1, testkD = 0, testkIz = 0, testkFF = 0.000040, testkMaxOutput = .85, testkMinOutput = -.85;

double ty = table->GetNumber("ty", 0.0);
double targetDist = (5.5 / tan((30 + ty) * (wpi::math::pi / 180)));

void testPIDcontroller(/*frc::XboxController * shooteroperater*/)
{
    //test motor PID
    double testMaxRPM = 5700;
    double distconversion = 100;
    double testSetPoint = 2500 + (round(targetDist) * distconversion);

    frc::SmartDashboard::PutNumber("distConversion", distconversion);

    /*bool testABtn;
    bool testBBtn;
    bool testXBtn;
    bool testYBtn;*/
    //double RPMstick;
    //test motor
  /*testABtn = shooteroperater->GetAButton();
  testBBtn = shooteroperater->GetBButton();
  testXBtn = shooteroperater->GetXButton();
  testYBtn = shooteroperater->GetYButton();
  RPMstick = shooteroperater->GetY(frc::GenericHID::JoystickHand::kLeftHand);*/
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
    /*if (testABtn)
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
    }*/
    // else
    // {
    //     m_testMotor.Set(0);
    // }
    /*if (-.1 > RPMstick || .1 < RPMstick || testXBtn || testYBtn || testBBtn || testABtn)
    {
        if (testSetPoint >= 1000)
        {
            testiz = testSetPoint - 1000;
        }
        else
        {
            testiz = 0;
        }
        m_testpidController.SetIZone(testiz);*/
        m_testpidController.SetReference(testSetPoint, rev::ControlType::kVelocity);
    /*}*/
    frc::SmartDashboard::PutNumber("Motor temps", m_testMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("SetPoint", testSetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_testEncoder.GetVelocity());
}