#include "ptac_control.h"
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26xx.h>

///// Pin Configurations/////
/////////////////////////////

//added this to the main.c file
   /*
    PIN_Config BoardGpioInitTable[] = {
         // DIO22:  (initially off)
         PIN_ID(22) | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
         // DIO21:  (initially off)
         PIN_ID(21)  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
         // DIO20:
         PIN_ID(20) | PIN_INPUT_EN  | PIN_PULLUP | PIN_HYSTERESIS,
         // Terminate list
         PIN_TERMINATE
     };
*/

///// Global variable initialize/////
/////////////////////////
int userSetTemp_global;
int currentTemp_global;
int heaterFunctionality_global;
int heat_relay=22;
int ac_relay=21;
int fan_relay=20;



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



////////////////////////
////////Functions///////
////////////////////////


// Relay Status ////////////////////
//receives relay name and requested action, if a 1 is received then the relay will turn on, zero will turn off
//

int relay_status_change(relay_name,requested_action)
{
    if(requested_action)
    {
        PIN_setOutputValue(relay_name, PIN_ID(relay_name), 1);
    }
    else
    {
        PIN_setOutputValue(relay_name, PIN_ID(relay_name), 0);
    }
    return 0;
}


// Temp Change //////////////////////
//Receives temperature information, decides if temperature needs to be turned on or off
// 0 = no change
// 1 = heat on
// 2 = AC and fan on

int temp_change(current,requested)
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




int action_request(heater_functionality_off,heat_relay_OR,ac_relay_OR,fan_relay_OR,temp_current,temp_set)
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



/*

int control()
{
int heat_func  = 0;
int heat_relay = 0;
int ac_relay   = 0;
int fan_relay  = 0;
int i = 1;
while(1)
    {
    if(i==1)
    {
        heat_func  = 1;
        heat_relay= 1;
        ac_relay    = 0;
        fan_relay  = 0;
    }
    else if(i==2)
    {
        heat_func  = 0;
        heat_relay = 1;
        ac_relay   = 0;
        fan_relay  = 0;
    }
    else if(i==3)
    {
        heat_func  = 0;
        heat_relay = 0;
        ac_relay   = 1;
        fan_relay  = 0;
    }
    else if(i==4)
    {
        heat_func   = 0;
        heat_relay  = 0;
        ac_relay    = 1;
        fan_relay   = 1;
    }
    else if(i==5)
    {
        heat_func  = 0;
        heat_relay = 0;
        ac_relay   = 0;
        fan_relay  = 1;
    }
    else
    {
        heat_func  = 0;
        heat_relay = 0;
        ac_relay   = 0;
        fan_relay  = 0;
        i = 0;
    }

    action_request(heat_func,heat_relay,ac_relay,fan_relay);
    delay(10000);
    i=i+1;
    }
    return 0;
}
*/
