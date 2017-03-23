#include "ptac_control.h"
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26xx.h>
#include <../Application/ptac_control_app.h>
#include "board.h"





  //PIN_setOutputEnable(&hStateHui, Board_DIO12, 1);

///// Global variable initialize/////
/////////////////////////
int userSetTemp_global;
int currentTemp_global;
int heaterFunctionality_global;

int heat_relay=Board_DIO12;
int ac_relay=Board_I2C0_SDA0;
int fan_relay=Board_I2C0_SCL0;

//PIN_State hstate_passed;

////////////////////////
////////Functions///////
////////////////////////




// Temp Change //////////////////////
//Receives temperature information, decides if temperature needs to be turned on or off
// 0 = no change
// 1 = heat on
// 2 = AC and fan on

int temp_change(int current,int requested)
{
int change_required;

    if (requested > current)            //cool
    {
        change_required = 1;
    }

    else if(requested < current)        //heat
    {
        change_required = 2;
    }
    else
    {
        change_required = 0;            //do nothing
    }
return change_required;
}

// Relay Status ////////////////////
//receives relay name and requested action, if a 1 is received then the relay will turn on, zero will turn off
//

void relay_status_change(int relay_name,int requested_action)
{

  //  PIN_State relay_handle =  hstate_passed;
    if(requested_action)
    {
        PIN_setOutputEnable(&hStateHui, relay_name, 1);
    }
    else
    {
        PIN_setOutputEnable(&hStateHui, relay_name, 0);
    }
return;
}




//////action request////////////////////
//When an the user inputs a temp change request or relay override request this function will turn on the required relay for the change
//Note: Heat can not be turned on if the heat functionality is turned off
/////////////////////////////////////////////
//////Possible requests from user below//////
//////set temperature
//////turn heat functionality off
//////heat relay override
//////ac relay override
//////fan relay override

void action_request(int heater_functionality_off,int heat_relay_OR, int ac_relay_OR,int fan_relay_OR,int temp_current,int temp_set)
{

//auto change temp
int action_change_temp;

action_change_temp = temp_change(currentTemp_global,userSetTemp_global);                     //Receive current temp and requested temp compare value, return required action


if(((heat_relay_OR)&(heater_functionality_off==0))||(action_change_temp==2))
    {
    relay_status_change(heat_relay,1);
    relay_status_change(fan_relay,0);
    relay_status_change(ac_relay,0);                                         //turn on relay based on required action
    }
else if((ac_relay_OR)||(action_change_temp==1))
    {
    relay_status_change(heat_relay,0);
    relay_status_change(fan_relay,1);
    relay_status_change(ac_relay,1);                                         //turn on relay based on required action
    }
else if (fan_relay_OR)
    {
    relay_status_change(fan_relay,1);
    }
else
    {
    relay_status_change(heat_relay,0);
    relay_status_change(fan_relay,0);
    relay_status_change(ac_relay,0);
    }

return;
}






void SetTemperature(uint8_t setTemperature)
{
    userSetTemp_global=setTemperature;
    action_request(heaterFunctionality_global,0,0,0,currentTemp_global,userSetTemp_global);
}

void ForceFan(uint8_t forceFan)
{
    action_request(heaterFunctionality_global,0,0,1,currentTemp_global,userSetTemp_global);
}

void ForceCool(uint8_t forceCool)
{
    action_request(heaterFunctionality_global,0,1,0,currentTemp_global,userSetTemp_global);
}

void ForceHeat(uint8_t forceHeat)
{
    action_request(heaterFunctionality_global,1,0,0,currentTemp_global,userSetTemp_global);
}

void UpdatePTAC(uint8_t actualTemperature)
{
    currentTemp_global=actualTemperature;
    action_request(heaterFunctionality_global,0,0,0,currentTemp_global,userSetTemp_global);
}


//pass hstate to ptac control
/*void hstate_pass(PIN_State handle)
{
     hstate_passed=handle;
return;
}
*/


