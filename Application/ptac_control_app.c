/*
 * Filename: multi_role.c
 *
 * Description: This file contains the multi_role sample application for use
 * with the CC2650 Bluetooth Low Energy Protocol Stack.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
* INCLUDES
*/
#include "../Application/ptac_control_app.h"

#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "att.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "../PROFILES/PTAC_control_profile.h"
#include "../PROFILES/thermometer_profile.h"
#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include <ti/mw/lcd/LCDDogm1286.h>
#include "../Application/ptac_control.h"


#define TI_UUID_SIZE        ATT_UUID_SIZE
#define TI_UUID(uuid)       TI_BASE_UUID_128(uuid)

/*********************************************************************
* CONSTANTS
*/
#define PERIODIC_EVT_PERIOD               5000

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

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   944
#endif

// Internal Events for RTOS application
#define MR_STATE_CHANGE_EVT                  0x0001
#define MR_CHAR_CHANGE_EVT                   0x0002
#define MR_CONN_EVT_END_EVT                  0x0004
#define MR_KEY_CHANGE_EVT                    0x0008
#define MR_PAIRING_STATE_EVT                 0x0010
#define MR_PASSCODE_NEEDED_EVT               0x0020
#define MR_PERIODIC_EVT                      0x0040

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_CHAR_DISC_STATE_CONFIG,         // Discover config
  BLE_CHAR_DISC_STATE_DATA,           // Discover data
  BLE_CHAR_DISC_STATE_PERIOD,         // Discover period
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
  uint16_t dataCharHdl;    //data characteristic handle
  uint16_t configCharHdl;  //data characteristic handle
  uint16_t periodCharHdl;  //data characteristic handle
  uint16 connectionHandle;
} discInfo_t;

typedef struct
{
    uint8_t *peeraddr;
    uint8_t addrtype;
} linkEstablishmentInfo;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

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
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

static uint8_t scanRspData[] =
{
  // complete name
  13,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'P', 'T', 'A', 'C', ' ', 'C', 'o', 'n', 't', 'r', 'o', 'l',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(PTACCONTROL_SERV_UUID),
  HI_UINT16(PTACCONTROL_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "PTAC Control";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Discovered thermometer info
static discInfo_t thermometerDiscInfo;
// Info for the link to be requested after scanning finishes
static linkEstablishmentInfo linkToMake;

// Maximim PDU size (default = 27 octets)
static uint16 maxPduSize;

// Scanning state
static bool scanningStarted = FALSE;

// Number of scan results and scan result index for menu functionality
static uint8_t scanRes;

// connecting state
static uint8_t connectingState = 0;

// Dummy parameters to use for connection updates
gapRole_updateConnParams_t updateParams =
{
  .connHandle = INVALID_CONNHANDLE,
  .minConnInterval = 80,
  .maxConnInterval = 150,
  .slaveLatency = 0,
  .timeoutMultiplier = 200
};

// maximum number of connected devices
static uint8_t maxNumBleConns = MAX_NUM_BLE_CONNS;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);

static void multi_role_startThermometerDiscovery(uint16_t connHandle);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
static bool multi_role_findLocalName(uint8_t *pName, uint8_t nameLength, uint8_t *pData, uint8_t dataLen);
//static void multi_role_handleKeys(uint8_t keys);
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_keyChangeHandler(uint8 keysPressed);
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData);
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void PTACControlApp_clockHandler(UArg arg);
static void PTACControlApp_performPeriodicTask(void);
static void PTACControl_initializeTemperatureSensor();
static void WriteChar(uint16 connectionHandle, uint16 charHandle, uint8 value);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,        // events to be handled by the app are passed through the GAP Role here
};

// Simple GATT Profile Callbacks
static ptacControlCBs_t multi_role_ptacControlCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs =
{
  (pfnPasscodeCB_t)multi_role_passcodeCB, // Passcode callback
  multi_role_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for the Simple BLE Peripheral.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;

  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void multi_role_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  Util_constructClock(&periodicClock, PTACControlApp_clockHandler,
                      PERIODIC_EVT_PERIOD, 0, false, MR_PERIODIC_EVT);

  //init keys and LCD
  Board_initKeys(multi_role_keyChangeHandler);
  // Open Display.
  //dispHandle = Display_open(SBC_DISPLAY_TYPE, NULL);

  // Setup the GAP
  {


    /*-------------------PERIPHERAL-------------------*/
    // set advertising interval the same for all scenarios
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);


    /*-------------------CENTRAL-------------------*/
    // set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
    // scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);
    // set connection parameters
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);

    //register to receive GAP and HCI messages
    GAP_RegisterForMsgs(selfEntity);
  }

  // Setup the GAP Role Profile
  {


    /*--------PERIPHERAL-------------*/
    uint8_t initialAdvertEnable = TRUE;
    uint16_t advertOffTime = 0;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // device starts advertising upon initialization
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);
    // set scan response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData, NULL);
    // set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);
    // set connection parameters
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval, NULL);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval, NULL);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency, NULL);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout, NULL);


    /*--------------CENTRAL-----------------*/
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    //set the max amount of scan responses
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES,
                         sizeof(uint8_t),
                         &scanRes, NULL);

    // Start the GAPRole and negotiate max number of connections
    VOID GAPRole_StartDevice(&multi_role_gapRoleCBs, &maxNumBleConns);

    thermometerDiscInfo.discState = BLE_DISC_STATE_IDLE;
    thermometerDiscInfo.svcEndHdl = 0;
    thermometerDiscInfo.svcStartHdl = 0;
    thermometerDiscInfo.connectionHandle = INVALID_CONNHANDLE;
  }

  //GATT
  {


    /*---------------------SERVER------------------------*/
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    PtacControl_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Setup Profile Characteristic Values
    {
      uint8_t charValue1 = 0;
      uint8_t charValue2 = 0;
      uint8_t charValue3 = 0;
      uint8_t charValue4 = 0;
      uint8_t charValue5 = 25;

      PtacControl_SetParameter(PTACCONTROL_SETTEMPERATURE, sizeof(uint8_t),
                                 &charValue1);
      PtacControl_SetParameter(PTACCONTROL_FORCEFAN, sizeof(uint8_t),
                                 &charValue2);
      PtacControl_SetParameter(PTACCONTROL_FORCEHEAT, sizeof(uint8_t),
                                 &charValue3);
      PtacControl_SetParameter(PTACCONTROL_FORCECOOL, sizeof(uint8_t),
                                 &charValue4);
      PtacControl_SetParameter(PTACCONTROL_ACTUALTEMPERATURE, sizeof(uint8_t),
                                 &charValue5);
    }

    // Register callback with SimpleGATTprofile
    PtacControl_RegisterAppCBs(&multi_role_ptacControlCBs);


    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);
  }

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8_t mitm = FALSE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    // set pairing mode
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    // set authentication requirements
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    // set i/o capabilities
    //GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    // set bonding requirements
    //GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

    // Register and start Bond Manager
    VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);
  }



  //Display_print0(dispHandle, 0, 0, "Scan ->");
  //Display_print0(dispHandle, 0, 0, "<- Next Option");
}

static void PTACControlApp_clockHandler(UArg arg)
{
    // Store the event.
    events |= arg;

    // Wake up the application.
    Semaphore_post(sem);
}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & MR_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              multi_role_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          multi_role_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }

      if (events & MR_PERIODIC_EVT)
      {
        events &= ~MR_PERIODIC_EVT;

        Util_startClock(&periodicClock);

        // Perform periodic application task
        PTACControlApp_performPeriodicTask();
      }
    }
  }
}

/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
  case GATT_MSG_EVENT:
    // Process GATT message
    safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
    break;

  case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
      case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Event
        break;

      default:
        break;
      }
    }
    break;

  case GAP_MSG_EVENT:
    multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
    break;

  default:
    // do nothing
    break;
  }

  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   MR_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    //Display_print1(dispHandle, 0, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }
  uint8 numActive = linkDB_NumActive();
  //Display_print1(dispHandle, 0, 0, "FC Violated: %d", numActive);
  //messages from GATT server
  if (numActive > 0)
  {
    if ((pMsg->method == ATT_READ_RSP) ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
          //Read error
      }
      else
      {
          //Successful read

          UpdatePTAC(pMsg->msg.readRsp.pValue[0]);
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {

      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {
          //Write Error
      }
      else
      {
          //Succesful write
      }
    }
    else if (thermometerDiscInfo.connectionHandle == pMsg->connHandle &&
            thermometerDiscInfo.discState != BLE_DISC_STATE_IDLE)
    {
      multi_role_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      multi_role_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      //Display_print1(dispHandle, 0, 0, "Rsp send retry:", rspTxRetry);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0, "Rsp sent, retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      //Display_print1(dispHandle, 0, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;

  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

//  case MR_KEY_CHANGE_EVT:
//    multi_role_handleKeys(*(pMsg->pData));
//    // Free the app data
//    ICall_free(pMsg->pData);
//    break;

  case MR_PAIRING_STATE_EVT:
    multi_role_processPairState((gapPairStateEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_PASSCODE_NEEDED_EVT:
    multi_role_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Central event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    // GAPRole started
  case GAP_DEVICE_INIT_DONE_EVENT:
    {
      //store max pdu size
      maxPduSize = pEvent->initDone.dataPktLen;

      //set device info characteristic
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);

      //Start discovery
      GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
      scanningStarted = TRUE;
    }
    break;

    // advertising started
  case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {

    }
    break;

    // advertising ended
  case GAP_END_DISCOVERABLE_DONE_EVENT:
    {

    }
    break;

    // a discovered device report
  case GAP_DEVICE_INFO_EVENT:
    {
        //if the thermometer profile was found

        uint8_t *pName = "CC2650 SensorTag";
        uint8_t nameLength = 9;
        if (multi_role_findLocalName(pName, nameLength, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen))
        {
            linkToMake.peeraddr = pEvent->deviceInfo.addr;
            linkToMake.addrtype = pEvent->deviceInfo.addrType;
        }
    }
    break;

    // end of discovery report
  case GAP_DEVICE_DISCOVERY_EVENT:
    {
      // discovery complete
      scanningStarted = FALSE;

      if(linkToMake.peeraddr != 0)
      {
          bStatus_t success = GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                DEFAULT_LINK_WHITE_LIST,
                                linkToMake.addrtype, linkToMake.peeraddr);
          //Display_print1(dispHandle, 0, 0, "Param Update %d", success);
      }
      else if(thermometerDiscInfo.connectionHandle == INVALID_CONNHANDLE)
      {
          //Start discovery
          GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
          scanningStarted = TRUE;
      }
    }
    break;

    // connection has been established
  case GAP_LINK_ESTABLISHED_EVENT:
    {
      // if successfully established
      if (pEvent->gap.hdr.status == SUCCESS)
      {
        //update connecting state
        connectingState = 0;

        //turn off advertising if no available links
        if (linkDB_NumActive() >= maxNumBleConns)
        {
          uint8_t advertEnabled = FALSE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
        }

        //start service discovery
        multi_role_startThermometerDiscovery(pEvent->linkCmpl.connectionHandle);

      }
      // if the connection was not successfully established
      else
      {

      }
    }
    break;

    // connection has been terminated
  case GAP_LINK_TERMINATED_EVENT:
    {
        //if the terminated link was the thermometer
        if(thermometerDiscInfo.connectionHandle == pEvent->linkTerminate.connectionHandle)
        {
            //Reset discovery info
            thermometerDiscInfo.configCharHdl = 0;
            thermometerDiscInfo.dataCharHdl = 0;
            thermometerDiscInfo.periodCharHdl = 0;
            thermometerDiscInfo.connectionHandle = INVALID_CONNHANDLE;
            //reset discovery state
            thermometerDiscInfo.discState= BLE_DISC_STATE_IDLE;
            if (linkDB_NumActive() < maxNumBleConns)
            {
                if(scanningStarted == FALSE)
                {
                    //Start discovery
                    GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
                    scanningStarted = TRUE;
                }

                //
                uint8_t adv_status;
                //get current advertising status
                GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv_status, NULL);
                //if we're currently advertising
                if (adv_status == FALSE)
                {
                  //turn off advertising
                  uint16 adv = TRUE;
                  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
                }
            }
        }
    }
    break;

    //a parameter update has occurred
  case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      //Display_print1(dispHandle, 0, 0, "Param Update %d", pEvent->linkUpdate.status);
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t newValue;

  //print new value depending on which characteristic was updated
  switch(paramID)
  {
  case PTACCONTROL_SETTEMPERATURE:
    // get new value
    PtacControl_GetParameter(PTACCONTROL_SETTEMPERATURE, &newValue);
    SetTemperature(newValue);

    break;

  case PTACCONTROL_FORCEFAN:
    // get new value
      PtacControl_GetParameter(PTACCONTROL_FORCEFAN, &newValue);
      ForceFan(newValue);


    break;

  case PTACCONTROL_FORCEHEAT:
    // get new value
    PtacControl_GetParameter(PTACCONTROL_FORCEHEAT, &newValue);
    ForceHeat(newValue);


    break;

  case PTACCONTROL_FORCECOOL:
    // get new value
      PtacControl_GetParameter(PTACCONTROL_FORCECOOL, &newValue);
      ForceCool(newValue);


    break;

  default:
    // should not reach here!
    break;
  }
}

/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  // allocate space for the message
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

  // if sucessfully allocated
  if (pMsg)
  {
    // fill up message
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }

  return FALSE;
}

/*********************************************************************
* @fn      multi_role_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
void multi_role_keyChangeHandler(uint8 keys)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    // store the key data
    *pData = keys;

    // Queue the event.
    multi_role_enqueueMsg(MR_KEY_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_startThermometerDiscovery
*
* @brief   Start service discovery.
*
* @return  none
*/
static void multi_role_startThermometerDiscovery(uint16_t connHandle)
{
  //exchange MTU request
  attExchangeMTUReq_t req;

  //update discovery state of this connection
  thermometerDiscInfo.discState= BLE_DISC_STATE_MTU;
  // Initialize cached handles
  thermometerDiscInfo.svcStartHdl = 0;
  thermometerDiscInfo.svcEndHdl = 0;
  thermometerDiscInfo.configCharHdl = 0;
  thermometerDiscInfo.dataCharHdl = 0;
  thermometerDiscInfo.periodCharHdl = 0;
  thermometerDiscInfo.connectionHandle = connHandle;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  bStatus_t status = GATT_ExchangeMTU(connHandle, &req, selfEntity);
  //Display_print1(dispHandle, 0, 0, "Param Update %d", status);
}

/*********************************************************************
* @fn      multi_role_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @return  none
*/
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (pMsg->connHandle == thermometerDiscInfo.connectionHandle)
  {
    //MTU update
    if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {

    }
    //if we've updated the MTU size
    else if (thermometerDiscInfo.discState == BLE_DISC_STATE_MTU)
    {
      // MTU size response received, discover simple BLE service
      if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
      {
//        uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(IRTEMPERATURE_SERV_UUID),
//        HI_UINT16(IRTEMPERATURE_SERV_UUID) };
        uint8_t irSensorServiceUUID[TI_UUID_SIZE] =
        {
          TI_UUID(IRTEMPERATURE_SERV_UUID),
        };
        //advanec state
        thermometerDiscInfo.discState= BLE_DISC_STATE_SVC;

        // Discovery thermometer service
        VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, irSensorServiceUUID, TI_UUID_SIZE,
                                           selfEntity);
      }
    }
    //if we're performing service discovery
    else if (thermometerDiscInfo.discState== BLE_DISC_STATE_SVC)
    {
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
          thermometerDiscInfo.svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
          thermometerDiscInfo.svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }

      // If procedure complete
      if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
           (pMsg->hdr.status == bleProcedureComplete))  ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        //if we've discovered the service
        if (thermometerDiscInfo.svcStartHdl != 0)
        {
          attReadByTypeReq_t req;

          // Discover characteristic
          thermometerDiscInfo.discState = BLE_CHAR_DISC_STATE_DATA;
          req.startHandle = thermometerDiscInfo.svcStartHdl;
          req.endHandle = thermometerDiscInfo.svcEndHdl;
          req.type.len = TI_UUID_SIZE;
          uint8_t irSensorDataUUID[TI_UUID_SIZE] =
          {
            TI_UUID(IRTEMPERATURE_DATA_UUID),
          };

          for(int i = 0; i < TI_UUID_SIZE; i++)
          {
              req.type.uuid[i] = irSensorDataUUID[i];
          }

          //send characteristic discovery request
          VOID GATT_ReadUsingCharUUID(pMsg->connHandle, &req, selfEntity);
        }
      }
    }
    //if we're discovering characteristics
    else if (thermometerDiscInfo.discState == BLE_CHAR_DISC_STATE_DATA)
    {
      // Characteristic found
      if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
          (pMsg->msg.readByTypeRsp.numPairs > 0))
      {
          thermometerDiscInfo.dataCharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                                   pMsg->msg.readByTypeRsp.pDataList[1]);

          attReadByTypeReq_t req;

          // Discover characteristic
          thermometerDiscInfo.discState = BLE_CHAR_DISC_STATE_CONFIG;
          req.startHandle = thermometerDiscInfo.svcStartHdl;
          req.endHandle = thermometerDiscInfo.svcEndHdl;
          req.type.len = TI_UUID_SIZE;
          uint8_t irSensorDataUUID[TI_UUID_SIZE] =
          {
            TI_UUID(IRTEMPERATURE_CONFIG_UUID),
          };

          for(int i = 0; i < TI_UUID_SIZE; i++)
          {
              req.type.uuid[i] = irSensorDataUUID[i];
          }

          //send characteristic discovery request
          VOID GATT_ReadUsingCharUUID(pMsg->connHandle, &req, selfEntity);
      }
    }
    //if we're discovering characteristics
    else if (thermometerDiscInfo.discState == BLE_CHAR_DISC_STATE_CONFIG)
    {
      // Characteristic found
      if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
          (pMsg->msg.readByTypeRsp.numPairs > 0))
      {
          thermometerDiscInfo.configCharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                                   pMsg->msg.readByTypeRsp.pDataList[1]);

          attReadByTypeReq_t req;

          // Discover characteristic
          thermometerDiscInfo.discState = BLE_CHAR_DISC_STATE_PERIOD;
          req.startHandle = thermometerDiscInfo.svcStartHdl;
          req.endHandle = thermometerDiscInfo.svcEndHdl;
          req.type.len = TI_UUID_SIZE;
          uint8_t irSensorDataUUID[TI_UUID_SIZE] =
          {
            TI_UUID(IRTEMPERATURE_PERIOD_UUID),
          };

          for(int i = 0; i < TI_UUID_SIZE; i++)
          {
              req.type.uuid[i] = irSensorDataUUID[i];
          }

          //send characteristic discovery request
          VOID GATT_ReadUsingCharUUID(pMsg->connHandle, &req, selfEntity);
      }
    }
  }
  //if we're discovering characteristics
  else if (thermometerDiscInfo.discState == BLE_CHAR_DISC_STATE_PERIOD)
  {
    // Characteristic found
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
        thermometerDiscInfo.periodCharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                                 pMsg->msg.readByTypeRsp.pDataList[1]);
        thermometerDiscInfo.discState = BLE_DISC_STATE_IDLE;

        PTACControl_initializeTemperatureSensor();
    }
  }
}

/*********************************************************************
* @fn      multi_role_findLocalName
*
* @brief   Find a given local name in an advertiser's scan response data.
*
* @return  TRUE if local name was found found
*/
static bool multi_role_findLocalName(uint8_t *pName, uint8_t nameLength, uint8_t *pData,
                                   uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  //find the end of data
  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_LOCAL_NAME_COMPLETE) ||
          (adType == GAP_ADTYPE_LOCAL_NAME_SHORT))
      {
        pData++;
        adLen--;

        int8_t i = 0;

        while(adLen >= 0 && pData < pEnd && i < nameLength)
        {
            if(pName[i] != pData[0])
            {
                break;
            }
            pData++;
            adLen--;
            i++;
        }

        if(i == nameLength)
        {
            return TRUE;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
* @fn      multi_role_findSvcUuid
*
* @brief   Find a given UUID in an advertiser's service UUID list.
*
* @return  TRUE if service UUID found
*/
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                   uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  //find the end of data
  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) ||
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;

        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/************************************************************************
* @fn      multi_role_pairStateCB
*
* @brief   Pairing state callback.
*
* @return  none
*/
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
  gapPairStateEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPairStateEvent_t))))
  {
    pData->connectionHandle = connHandle;
    pData->state = state;
    pData->status = status;

    // Enqueue the event.
    multi_role_enqueueMsg(MR_PAIRING_STATE_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_passcodeCB
*
* @brief   Passcode callback.
*
* @return  none
*/
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    multi_role_enqueueMsg(MR_PASSCODE_NEEDED_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_processPairState
*
* @brief   Process the new paring state.
*
* @return  none
*/
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent)
{
  //if we've started pairing
  if (pairingEvent->state == GAPBOND_PAIRING_STATE_STARTED)
  {
    //Display_print1(dispHandle, 0, 0,"Cxn %d pairing started", pairingEvent->connectionHandle);
  }
  //if pairing is finished
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0,"Cxn %d pairing success", pairingEvent->connectionHandle);
    }
    else
    {
      //Display_print2(dispHandle, 0, 0, "Cxn %d pairing fail: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
  //if a bond has happened
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0, "Cxn %d bonding success", pairingEvent->connectionHandle);
    }
  }
  //if a bond has been saved
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
    }
    else
    {
      //Display_print2(dispHandle, 0, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
}

/*********************************************************************
* @fn      multi_role_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  //use static passcode
  uint32_t passcode = 123456;
  //Display_print1(dispHandle, 0, 0, "Passcode: %d", passcode);
  //send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      PTACControlApp_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every PERIODIC_EVT_PERIOD milliseconds.  It reads the temperature
 *          from the thermometer.
 *
 * @param   None.
 *
 * @return  None.
 */
static void PTACControlApp_performPeriodicTask(void)
{
    // create read request...place in CSTACK
    attReadReq_t req;

    // fill up read request
    req.handle = thermometerDiscInfo.dataCharHdl;

    //send read request. no need to free if unsuccesful since the request
    //is only placed in CSTACK; not allocated
    GATT_ReadCharValue(thermometerDiscInfo.connectionHandle, &req, selfEntity);
}

static void PTACControl_initializeTemperatureSensor()
{
    WriteChar(thermometerDiscInfo.connectionHandle, thermometerDiscInfo.configCharHdl, 0x01);
}

static void WriteChar(uint16 connectionHandle, uint16 charHandle, uint8 value)
{
    // Do a write
    attWriteReq_t req;

    //allocate GATT write request
    req.pValue = GATT_bm_alloc(connectionHandle, ATT_WRITE_REQ, 1, NULL);
    // if succesfully allocated
    if ( req.pValue != NULL )
    {
      //fill up request
      req.handle = charHandle;
      req.len = 1;
      req.pValue[0] = value;
      req.sig = 0;
      req.cmd = 0;

      //send GATT write to controller
      bStatus_t status = GATT_WriteCharValue(connectionHandle, &req, selfEntity);
      //if not sucessfully sent
      if ( status != SUCCESS )
      {
        //free write request as the controller will not
        GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
      }
    }
}


////use dummy params statically allocated but fill in connection handle
//updateParams.connHandle = connHandleMap[connIdx];
////send connection parameter update
//status = gapRole_connUpdate( GAPROLE_NO_ACTION, &updateParams);

/*********************************************************************
*********************************************************************/
