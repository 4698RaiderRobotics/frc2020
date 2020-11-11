#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

//test motor
double testkP = 0.000300, testkI = 1, testkD = 0, testkIz = 0, testkFF = 0.000040, testkMaxOutput = .85, testkMinOutput = -.85;
std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double testSetPoint = 0;
bool shootready;
bool rampupflag = true;

void testPIDcontroller(frc::XboxController *shooteroperater, rev::CANSparkMax *m_leadmotor, bool autoShoot, double targetdist)
{
    rev::CANPIDController m_testpidController = m_leadmotor->GetPIDController();
    // set test PID coefficients
    m_testpidController.SetP(testkP);
    m_testpidController.SetI(testkI * 1e-6);
    m_testpidController.SetD(testkD);
    m_testpidController.SetIZone(testkIz);
    m_testpidController.SetFF(testkFF);
    m_testpidController.SetOutputRange(testkMinOutput, testkMaxOutput);
    rev::CANEncoder m_testEncoder = m_leadmotor->GetEncoder();
    //test motor PID
    double testMaxRPM = 5700;
    double RPMstick;
    double povsetpoint;
    //test motor
    povsetpoint = shooteroperater->GetPOV();
    RPMstick = shooteroperater->GetY(frc::GenericHID::JoystickHand::kLeftHand);
    bool autodist = shooteroperater->GetXButton();
    bool autodistrelease = shooteroperater->GetXButtonReleased();
    double setpointup = shooteroperater->GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand);
    double setpointdown = shooteroperater->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
    bool setpointzero = shooteroperater->GetBackButtonPressed();
    bool shooterthrottle = shooteroperater->GetBumper(frc::GenericHID::JoystickHand::kLeftHand);

    //using targetDist
    double ty = table->GetNumber("ty", 0.0);
    double targetDist = (5.6666666 / tan((30.379 + ty) * (wpi::math::pi / 180)));
    //tuned to heavy flywheel
    double targetdistrpm = (27.98148 * targetDist) + 2269.41566;
    //tuned to light flywheel
    //double targetdistrpm = (29.07294 * targetdist) + 2504.59405;
    // read PID coefficients from SmartDashboard
    double testp = frc::SmartDashboard::GetNumber("test P Gain", 0.000250);
    double testi = frc::SmartDashboard::GetNumber("test I Gain", 0);
    double testd = frc::SmartDashboard::GetNumber("test D Gain", .000800);
    double testiz = frc::SmartDashboard::GetNumber("test I Zone", 0);
    double testff = frc::SmartDashboard::GetNumber("test Feed Forward", 0.000015);
    double testmax = frc::SmartDashboard::GetNumber("test Max Output", 0.85);
    double testmin = frc::SmartDashboard::GetNumber("test Min Output", -0.85);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if (testp != testkP)
    {
        m_testpidController.SetP(testp);
        testkP = testp;
    }
    if ((testi != testkI))
    {
        m_testpidController.SetI(testi * 1e-6);
        testkI = testi;
    }
    if (testd != testkD)
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
    if(shooterthrottle){    
        if (abs(m_testEncoder.GetVelocity()) <= 2400  && rampupflag) {
            m_leadmotor->Set(-.5);
        }
        else if(abs(m_testEncoder.GetVelocity()) >= 2400){
            rampupflag = false;
        }
        frc::SmartDashboard::PutBoolean("rampupflag", rampupflag);
        if (rampupflag == false){
            if (autodist || autoShoot)
            {
                testSetPoint = targetdistrpm;
                printf("autorpm %f \n", targetdistrpm);
            }
            else if(povsetpoint <= 135 && povsetpoint != -1){
                testSetPoint = 2560;
            }
            else if(povsetpoint >= 225){
                testSetPoint = 2910;
            }
            // else if(autodistrelease){
            //     rampupflag = true;
            // }
            if (autoShoot || povsetpoint != -1 || autodist)
            {
                if (testSetPoint >= 1000)
                {
                    testiz = testSetPoint - 1000;
                }
                else
                {
                    testiz = 0;
                }
                //data collection
                // printf("backbutton %u \n", setpointzero);
                // if (setpointup >= .1){
                //     testSetPoint = testSetPoint + 10;
                // }
                // if (setpointdown >= .1){
                //     testSetPoint = testSetPoint - 10;
                // }
                // if (setpointzero){
                //     testSetPoint = 0;
                // }
                //     if(testSetPoint == 0){
                //         m_leadmotor->Set(0);
                //     }
                //     else{
                m_testpidController.SetIZone(testiz);
                m_testpidController.SetReference(-testSetPoint, rev::ControlType::kVelocity);
            }
        }
        if (abs(testSetPoint + m_testEncoder.GetVelocity()) < 200)
        {
            shootready = true;
        }
        else
        {
            shootready = false;
        }
    }
    else{
        m_leadmotor->Set(-.1);
    }
    if(autodistrelease){
    rampupflag = true;
    }
    frc::SmartDashboard::PutBoolean("shoot ready", shootready);
    frc::SmartDashboard::PutNumber("Motor temps", m_leadmotor->GetMotorTemperature());
    frc::SmartDashboard::PutNumber("SetPoint", testSetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_testEncoder.GetVelocity());
}