#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

//Limit Switches
frc::DigitalInput firstSwitch{0};
frc::DigitalInput secondSwitch{1};
frc::DigitalInput thirdSwitch{2};
//distance sensor
frc::AnalogInput index1{3};
frc::AnalogInput index2{2};
//Limit Switch bools
bool motorOn = 0;
int ballCounter = 0;
bool firstdisabled = 0;
bool seconddisabled = 0;
bool thirddisabled = 0;
//declaring victor variable
VictorSPX vspx = /*device ID*/{8};

void indexing(bool auotonomous)
{
  //victor motor movement
  //vspx.Set(ControlMode::PercentOutput, .8);
    
  bool firstInput; //= firstSwitch.Get();
  bool secondInput; //= secondSwitch.Get();
  bool thirdInput; //= thirdSwitch.Get();

  //Best spacing is 20, but autonomous needs more
  double spacingval;

  double index1volt = index1.GetVoltage();
  double index3volt = index2.GetVoltage();
  //printf("output volts %f \n", index3volt);
  //printf("output volts %f \n", index2volt);
  printf("indexing counter %f \n", ballCounter);


  if (index1volt < 1.9){
    firstInput = false;
  }
  if (index1volt > 2.4){
    firstInput = true;
  }

  if (auotonomous == false){
    spacingval = 20;
  }
  if (auotonomous == true){
    spacingval = 30;
  }
  

  //idexing with 1 sensor
  if (firstInput == true && firstdisabled == false){
    vspx.Set(ControlMode::PercentOutput, .5);
    ballCounter = 1;
    firstdisabled == true;
  }
  if (ballCounter < spacingval && ballCounter != 0){
    vspx.Set(ControlMode::PercentOutput, .5);
    ballCounter ++;
  }
  if (ballCounter == spacingval){
    ballCounter = 0;
    vspx.Set(ControlMode::PercentOutput, 0);
    firstdisabled = false;
  }

  // printf("firstSwitch %u\n", firstInput);
  // printf("secondSwitch %u\n", secondInput);
  // printf("thirdSwitch %u\n", thirdInput);
  //first switch on
  /*if (firstInput == 1 && firstdisabled == 0)
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
  */


}