/*
 * pin_control.c
 *
 *  Created on: Mar 12, 2017
 *      Author: Jason
 */

#ifndef APPLICATION_PIN_CONTROL_C_
#define APPLICATION_PIN_CONTROL_C_





#endif /* APPLICATION_PIN_CONTROL_C_ */

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "../PROFILES/PTAC_control_profile.h"
#include "../PROFILES/thermometer_profile.h"
#include "PTAC_control.h"

#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "ptac_control_app.h"

#include <ti/mw/lcd/LCDDogm1286.h>


#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26xx.h>


#include "pin_control.h"
/*********************************************************************
* CONSTANTS
*/
#define PERIODIC_EVT_PERIOD               1000


// Task configuration
#define RC_TASK_PRIORITY                     1
#ifndef RC_TASK_STACK_SIZE
#define RC_TASK_STACK_SIZE                   944
#endif

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters if automatic  parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

//connection parameters
#define DEFAULT_CONN_INT                      200
#define DEFAULT_CONN_TIMEOUT                  1000
#define DEFAULT_CONN_LATENCY                  0

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 4000
#define DEFAULT_SCAN_WIND                     80
#define DEFAULT_SCAN_INT                      80

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
#if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
#define SBC_DISPLAY_TYPE Display_Type_LCD
#elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
#define SBC_DISPLAY_TYPE Display_Type_UART
#else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
#define SBC_DISPLAY_TYPE 0 // Option not supported
#endif // BOARD_DISPLAY_EXCLUDE_LCD
#else // Display_DISABLE_ALL
#define SBC_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL


// Internal Events for RTOS application
#define RC_STATE_CHANGE_EVT                  0x0001
#define RC_CHAR_CHANGE_EVT                   0x0002
#define RC_CONN_EVT_END_EVT                  0x0004
#define RC_KEY_CHANGE_EVT                    0x0008
#define RC_PAIRING_STATE_EVT                 0x0010
#define RC_PASSCODE_NEEDED_EVT               0x0020
#define RC_PERIODIC_EVT                      0x0040

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t *pData; // event data pointer
} mrEvt_t;

// pairing callback event
typedef struct
{
  uint16 connectionHandle; //!< connection Handle
  uint8 state;             //!< state returned from GAPBondMgr
  uint8 status;            //!< status of state
} gapPairStateEvent_t;

// discovery information
typedef struct
{
  discState_t discState;   //discovery state
  uint16_t svcStartHdl;    //service start handle
  uint16_t svcEndHdl;      //service end handle
  uint16_t charHdl;        //characteristic handle
  uint16 connectionHandle;
} discInfo_t;


/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
//Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;


// Clock for PTAC control updates
static Clock_Struct periodicClock;
// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct RCTask;
Char RCTaskStack[RC_TASK_STACK_SIZE];







/*********************************************************************
* LOCAL FUNCTIONS
*/
void relay_control_createTask(void);
static void PTACPINCONTROL_performPeriodicTask(void);
static void RelayControl_taskFxn(UArg a0, UArg a1);
//clock handler function
static void RelayControl_clockHandler(UArg arg);
static void RelayControl_init(void);












void relay_control_createTask(void)
{
Task_Params taskParams;
// Configure task
Task_Params_init(&taskParams);
taskParams.stack = RCTaskStack;
taskParams.stackSize = RC_TASK_STACK_SIZE;
taskParams.priority = RC_TASK_PRIORITY;

Task_construct(&RCTask, RelayControl_taskFxn, &taskParams, NULL);
}




//clock handler function
static void
RelayControl_clockHandler(UArg arg)
{
//Store the event.
events |= arg;
//Wake up the application.
Semaphore_post(sem);
}


PIN_State hStateHui;
#define Heat_relay_pin  PIN_ID(31)
#define Cool_relay_pin  PIN_ID(9)
#define Fan_relay_pin   PIN_ID(10)


static void RelayControl_taskFxn(UArg a0, UArg a1)
{
    Util_startClock(&periodicClock);
    const PIN_Config aPinListHui[] = {
         Heat_relay_pin | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,                                 /* GPIO Pin 22  */
         Cool_relay_pin | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,                                 /* GPIO Pin 21  */
         Fan_relay_pin  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,                                 /* GPIO Pin 20  */
         PIN_TERMINATE
      };

    // Get handle to this collection of pins
      if (!PIN_open(&hStateHui, aPinListHui)) {
          // Handle allocation error
      }


        ////test purposes////

        PIN_setOutputValue(&hStateHui, Heat_relay_pin, 1);




        PIN_close(&hStateHui);

    RelayControl_init();
  // Initialize application



    Display_print1(0, 0, 0, "FC Violated: %d", 0);


    for (;;)
      {
        //handle event in application task handler
        if (events & RC_PERIODIC_EVT)
        {
        events &= ~RC_PERIODIC_EVT;
        PTACPINCONTROL_performPeriodicTask();
        Util_startClock(&periodicClock);
        // Perform periodic application task

        }

      }
}
//Create one-shot clocks for internal periodic events.
//Util_constructClock(&periodicClock, RelayControl_clockHandler,
//PERIODIC_EVT_PERIOD, 0, false, RC_PERIODIC_EVT);
static void RelayControl_init(void)
{
    ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  Util_constructClock(&periodicClock, RelayControl_clockHandler,
                      PERIODIC_EVT_PERIOD, 0, false, RC_PERIODIC_EVT);

  //init keys and LCD



//  Util_startClock(&periodicClock);

}








static void PTACPINCONTROL_performPeriodicTask(void)
{
 }
