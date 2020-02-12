#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "rev/CANSparkMax.h"
#include <frc/util/color.h>

//Color Sensor
static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;
static constexpr frc::Color kBlueTarget = frc::Color(0.197, 0.464, 0.337);
static constexpr frc::Color kGreenTarget = frc::Color(0.220, 0.512, 0.267);
static constexpr frc::Color kRedTarget = frc::Color(0.324, 0.443, 0.231);
static constexpr frc::Color kYellowTarget = frc::Color(0.280, 0.521, 0.198);

//Color Sensor
bool startbtn;
bool position;
bool rotationControl;
std::string gameData;
frc::Color matchedColor;
std::string currentColor;
std::string initColor;
std::string lastColor;
int revs = 0;
bool initp = true;
bool initr = true;

void readColorSensor()
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