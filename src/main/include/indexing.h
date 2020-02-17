#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

//Limit Switches
frc::DigitalInput firstSwitch{0};
frc::DigitalInput secondSwitch{1};
frc::DigitalInput thirdSwitch{2};
//Limit Switch bools
bool motorOn = 0;
double ballCounter = 0;
bool firstdisabled = 0;
bool seconddisabled = 0;
bool thirddisabled = 0;
//declaring victor variable
VictorSPX vspx = /*device ID*/{8};

void indexing()
{
  //victor motor movement
  //vspx.Set(ControlMode::PercentOutput, .8);
    
  bool firstInput = firstSwitch.Get();
  bool secondInput = secondSwitch.Get();
  bool thirdInput = thirdSwitch.Get();

  printf("firstSwitch %u\n", firstInput);
  printf("secondSwitch %u\n", secondInput);
  printf("thirdSwitch %u\n", thirdInput);
  //first switch on
  if (firstInput == 1 && firstdisabled == 0)
  {
    
    motorOn = 1;
    firstdisabled = 1;
  }
  //second switch on
  if (secondInput == 1 && seconddisabled == 0)
  {
    
    motorOn = 0;
    seconddisabled = 1;
  }
  //second switch released
  if (secondInput == 0 && seconddisabled == 1)
  {
    seconddisabled = 0;
  }

  //third switch on
  if (thirdInput == 1 && thirddisabled == 0)
  {
    
    thirddisabled = 1;
  }

  //third disabled
  if (thirdInput == 0 && thirddisabled == 1)
  {
    ballCounter--;
    thirddisabled = 0;
  }

  //first disabled
  if (firstInput == 0 && firstdisabled == 1)
  {
    ballCounter++;
    firstdisabled = 0;
  }

  if (motorOn == 1)
  {
    printf("motor running\n");
    vspx.Set(ControlMode::PercentOutput, .5);
  }
  if (motorOn == 0)
  {
    printf("stop motors\n");
    vspx.Set(ControlMode::PercentOutput, 0);
  }

  frc::SmartDashboard::PutNumber("ballCounter", ballCounter);

}