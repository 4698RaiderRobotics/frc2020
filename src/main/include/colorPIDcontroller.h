#include "rev/CANSparkMax.h"
//#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

//Color Sensor Motor and Encoder
rev::CANSparkMax m_colorwheel{9, rev::CANSparkMax::MotorType::kBrushless};
rev::CANPIDController m_colorPIDcontroller = m_colorwheel.GetPIDController();
rev::CANEncoder m_colorencoder = m_colorwheel.GetEncoder();
double colorkP = 6e-5, colorkI = 1e-6, colorkD = 0, colorkIz = 0, colorkFF = 0.000015, colorkMaxOutput = 0.8, colorkMinOutput = -0.8;
void colorPIDcontroller(double colorSetPoint)
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