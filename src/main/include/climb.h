#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

rev::CANSparkMax m_climbright{15, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_climbleft{16, rev::CANSparkMax::MotorType::kBrushless};

rev::CANPIDController m_leftPID = m_climbleft.GetPIDController();
rev::CANEncoder m_leftencoder = m_climbleft.GetEncoder();

rev::CANPIDController m_rightPID = m_climbright.GetPIDController();
rev::CANEncoder m_rightencoder = m_climbright.GetEncoder();

double leftclimbkP = 6e-5, leftclimbkI = 1e-6, leftclimbkD = 0, leftclimbkIz = 0, leftclimbkFF = 0.000015, leftclimbkMaxOutput = 1.0, leftclimbkMinOutput = -1.0;
double joystickcontrol;
bool maxheight;
double heightinches = 54;

void climber(frc::XboxController *driverclimb){
    m_leftPID.SetP(leftclimbkP);
    m_leftPID.SetI(leftclimbkI * 1e-6);
    m_leftPID.SetD(leftclimbkD);
    m_leftPID.SetIZone(leftclimbkIz);
    m_leftPID.SetFF(leftclimbkFF);
    m_leftPID.SetOutputRange(leftclimbkMinOutput, leftclimbkMaxOutput);
    m_leftencoder.SetPositionConversionFactor(.01);
    m_rightencoder.SetPositionConversionFactor(.01);

    joystickcontrol = driverclimb->GetY(frc::GenericHID::JoystickHand::kRightHand);
    maxheight = driverclimb->GetYButton();
    printf("%f\n", m_leftencoder.GetPosition());
    if(maxheight){
        if(m_leftencoder.GetPosition() <= 25){
            m_climbleft.Set(1);
            m_climbright.Set(1);
        }
        else{
            m_climbleft.Set(0);
            m_climbright.Set(0);
        }
    }
    else{
        m_climbleft.Set(joystickcontrol);
        if(m_leftencoder.GetPosition() <= m_rightencoder.GetPosition()){
            
        }
    }
}