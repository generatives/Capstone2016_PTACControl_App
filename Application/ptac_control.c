


/*********************************************************************
* INCLUDES
*/
#include "ptac_control.h"
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26xx.h>
#include <../Application/ptac_control_app.h>
#include "board.h"

#define HEAT_PIN Board_I2C0_SDA0
#define COOL_PIN Board_I2C0_SCL0
#define FAN_PIN Board_DIO21

#define HEAT_ONLY 2
#define AC_ONLY 1
#define AUTO 0

/*********************************************************************
* CONSTANTS
*/



/*********************************************************************
* TYPEDEFS
*/




/*********************************************************************
* GLOBAL VARIABLES
*/

int userSetTemp_global;
int currentTemp_global;
int heaterFunctionality_global;
int forceMode;
int forceFanOR;

#define heat_relay  Board_DIO12
#define ac_relay    Board_I2C0_SDA0
#define fan_relay   Board_I2C0_SCL0



/*********************************************************************
* LOCAL VARIABLES
*/



/*********************************************************************
* LOCAL FUNCTIONS
*/
int temp_change(int current,int requested);
void relay_status_change(int relay_name,int requested_action);
void action_request(int forceMode ,int fan_relay_OR,int temp_current,int temp_set);
void SetTemperature(uint8_t setTemperature);
void ForceFan(uint8_t forceFan);
void ForceCool(uint8_t forceCool);
void ForceHeat(uint8_t forceHeat);
void UpdatePTAC(uint8_t actualTemperature);


/*********************************************************************
* PROFILE CALLBACKS
*/




/*********************************************************************
* PUBLIC FUNCTIONS
*/


/*********************************************************************
* @fn      Temp Change
*
* @brief   Receives temperature information, decides if temperature needs to be turned on or off
*            0 = no change
*            1 = heat on
*            2 = AC and fan on
* @param  current tempurature, requested temperature.
*
* @return  change_required.
*/

int temp_change(int current,int requested)
{
int change_required;

    if (requested > current)            //heat
    {
        change_required = 2;
    }

    else if(requested < current)        //cool
    {
        change_required = 1;
    }
    else
    {
        change_required = 0;            //do nothing
    }
return change_required;
}



/*********************************************************************
* @fn      Relay Status
*
* @brief   receives relay name and requested action, if a 1 is received then the relay will turn on, zero will turn off
*
* @param  relay name, requested action.
*
* @return  none.
*/

/*
void relay_status_change(int relay_name,int requested_action)
{

  //  PIN_State relay_handle =  hstate_passed;
    if(requested_action)
    {
        PIN_setOutputValue(&hStateHui, relay_name, 1);
    }
    else
    {
        PIN_setOutputValue(&hStateHui, relay_name, 0);
    }
return;
}
*/


/*********************************************************************
* @fn     action_request
*
* @brief   When an the user inputs a temp change request or relay override request this function will turn on the required relay for the change
*          Note: Heat can not be turned on if the heat functionality is turned off
*
*          Possible requests:
*          set temperature
*          turn heat functionality off
*          heat relay override
*          ac relay override
*          fan relay override
*
* @param  relay name, requested action.
*
* @return  none.
*/

void action_request(int forceMode,int fan_relay_OR,int temp_current,int temp_set)
{

//auto change temp
int action_change_temp=0;

action_change_temp = temp_change(currentTemp_global,userSetTemp_global);                     //Receive current temp and requested temp compare value, return required action


// Define pins used by Human user interface and initial configuration

// Get handle to this collection of pins
//if (!PIN_open(&hStateHui, aPinListHui)) {
//    PIN_setPortOutputEnable(&hStateHui, 1);
//}

    if(forceMode != AC_ONLY && action_change_temp==2)
    {
        PIN_setOutputValue(&hStateHui, HEAT_PIN , 1);
        PIN_setOutputValue(&hStateHui, COOL_PIN , 0);
        PIN_setOutputValue(&hStateHui, FAN_PIN , 1);
    }
    else if(forceMode != HEAT_ONLY && (action_change_temp==1))
    {
        PIN_setOutputValue(&hStateHui, HEAT_PIN , 0);
        PIN_setOutputValue(&hStateHui, COOL_PIN , 1);
        PIN_setOutputValue(&hStateHui, FAN_PIN , 1);
    }
    else if (fan_relay_OR)
    {
        PIN_setOutputValue(&hStateHui, HEAT_PIN , 0);
        PIN_setOutputValue(&hStateHui, COOL_PIN , 0);
        PIN_setOutputValue(&hStateHui, FAN_PIN , 1);
    }
    else
    {
        PIN_setOutputValue(&hStateHui, HEAT_PIN , 0);
        PIN_setOutputValue(&hStateHui, COOL_PIN , 0);
        PIN_setOutputValue(&hStateHui, FAN_PIN , 0);
    }
//PIN_close(&hStateHui);

return;
}



/*********************************************************************
* @fn     SetTemperature
*
* @brief

*
* @param setTemperature.
*
* @return  none.
*/

void SetTemperature(uint8_t setTemperature)
{
    userSetTemp_global=setTemperature;
    action_request(forceMode,forceFanOR,currentTemp_global,userSetTemp_global);
}

/*********************************************************************
* @fn     ForceFan
*
* @brief
*
* @param forcefan.
*
* @return  none.
*/

void ForceFan(uint8_t forceFan)
{
    forceFanOR = forceFan;
    action_request(forceMode,forceFanOR,currentTemp_global,userSetTemp_global);
}

/*********************************************************************
* @fn     ForceHeat
*
* @brief
*
* @param forceheat.
*
* @return  none.
*/

void ForceMode(uint8_t forceModeParam)
{
    forceMode = forceModeParam;
    action_request(forceMode,forceFanOR,currentTemp_global,userSetTemp_global);
}


/*********************************************************************
* @fn     UpdatePTAC
*
* @brief
*
* @param actualTemperature.
*
* @return  none.
*/

void UpdatePTAC(uint8_t actualTemperature)
{
    currentTemp_global=actualTemperature;
    action_request(forceMode,forceFanOR,currentTemp_global,userSetTemp_global);
}



