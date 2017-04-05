/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 Copyright (c) 2010-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "osal.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "PTAC_control_profile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 ptacControlServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PTACCONTROL_SERV_UUID), HI_UINT16(PTACCONTROL_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 ptacControlSetTemperatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PTACCONTROL_SETTEMPERATURE_UUID), HI_UINT16(PTACCONTROL_SETTEMPERATURE_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 ptacControlForceFanUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PTACCONTROL_FORCEFAN_UUID), HI_UINT16(PTACCONTROL_FORCEFAN_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 ptacControlForceModeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PTACCONTROL_FORCEMODE_UUID), HI_UINT16(PTACCONTROL_FORCEMODE_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 ptacControlActualTemperatureUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PTACCONTROL_ACTUALTEMPERATURE_UUID), HI_UINT16(PTACCONTROL_ACTUALTEMPERATURE_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static ptacControlCBs_t *ptacControl_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t ptacControlService = { ATT_BT_UUID_SIZE, ptacControlServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 ptacControlSetTemperatureProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 ptacControlSetTemperature = 0;

// Simple Profile Characteristic 1 User Description
static uint8 ptacControlSetTemperatureUserDesp[15] = "Set Temperature";


// Simple Profile Characteristic 2 Properties
static uint8 ptacControlForceFanProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 2 Value
static uint8 ptacControlForceFan = 0;

// Simple Profile Characteristic 2 User Description
static uint8 ptacControlForceFanUserDesp[9] = "Force Fan";


// Simple Profile Characteristic 3 Properties
static uint8 ptacControlForceModeProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 ptacControlForceMode = 0;

// Simple Profile Characteristic 3 User Description
static uint8 ptacControlForceModeUserDesp[10] = "Force Mode";


// Simple Profile Characteristic 4 Properties
static uint8 ptacControlActualTemperatureProps = GATT_PROP_READ;

// Characteristic 4 Value
static uint8 ptacControlActualTemperature = 0;

// Simple Profile Characteristic 4 User Description
static uint8 ptacControlActualTemperatureUserDesp[18] = "Actual Temperature";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t ptacControlAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // Simple Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&ptacControlService            /* pValue */
  },

    // Characteristic 1 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ptacControlSetTemperatureProps
    },

      // Characteristic Value 1
      {
        { ATT_BT_UUID_SIZE, ptacControlSetTemperatureUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &ptacControlSetTemperature
      },

      // Characteristic 1 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        ptacControlSetTemperatureUserDesp
      },

    // Characteristic 2 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ptacControlForceFanProps
    },

      // Characteristic Value 2
      {
        { ATT_BT_UUID_SIZE, ptacControlForceFanUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &ptacControlForceFan
      },

      // Characteristic 2 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        ptacControlForceFanUserDesp
      },

    // Characteristic 3 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ptacControlForceModeProps
    },

      // Characteristic Value 3
      {
        { ATT_BT_UUID_SIZE, ptacControlForceModeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &ptacControlForceMode
      },

      // Characteristic 3 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        ptacControlForceModeUserDesp
      },

    // Characteristic 5 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ptacControlActualTemperatureProps
    },

      // Characteristic Value 5
      {
        { ATT_BT_UUID_SIZE, ptacControlActualTemperatureUUID },
        GATT_PERMIT_READ,
        0,
        &ptacControlActualTemperature
      },

      // Characteristic 5 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        ptacControlActualTemperatureUserDesp
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t ptacControl_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
static bStatus_t ptacControl_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t ptacControlCBs =
{
  ptacControl_ReadAttrCB,  // Read callback function pointer
  ptacControl_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      PtacControl_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t PtacControl_AddService( uint32 services )
{
  uint8 status;

  if ( services & PTACCONTROL_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( ptacControlAttrTbl,
                                          GATT_NUM_ATTRS( ptacControlAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &ptacControlCBs );
  }
  else
  {
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn      PtacControl_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t PtacControl_RegisterAppCBs( ptacControlCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    ptacControl_AppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      PtacControl_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t PtacControl_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PTACCONTROL_SETTEMPERATURE:
      if ( len == sizeof ( uint8 ) )
      {
        ptacControlSetTemperature = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case PTACCONTROL_FORCEFAN:
      if ( len == sizeof ( uint8 ) )
      {
        ptacControlForceFan = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case PTACCONTROL_FORCEMODE:
      if ( len == sizeof ( uint8 ) )
      {
        ptacControlForceMode = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case PTACCONTROL_ACTUALTEMPERATURE:
      if ( len == sizeof ( uint8 ) )
      {
        ptacControlActualTemperature = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      PtacControl_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t PtacControl_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PTACCONTROL_SETTEMPERATURE:
      *((uint8*)value) = ptacControlSetTemperature;
      break;

    case PTACCONTROL_FORCEFAN:
      *((uint8*)value) = ptacControlForceFan;
      break;

    case PTACCONTROL_FORCEMODE:
      *((uint8*)value) = ptacControlForceMode;
      break;

    case PTACCONTROL_ACTUALTEMPERATURE:
        *((uint8*)value) = ptacControlActualTemperature;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          ptacControl_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t ptacControl_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // all characteristics have read permissions
      case PTACCONTROL_SETTEMPERATURE_UUID:
      case PTACCONTROL_FORCEFAN_UUID:
      case PTACCONTROL_FORCEMODE_UUID:
      case PTACCONTROL_ACTUALTEMPERATURE_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      ptacControl_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t ptacControl_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
        case PTACCONTROL_SETTEMPERATURE_UUID:
        case PTACCONTROL_FORCEFAN_UUID:
        case PTACCONTROL_FORCEMODE_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 1 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          if(pAttr->pValue == &ptacControlSetTemperature)
          {
            notifyApp = PTACCONTROL_SETTEMPERATURE;
          }
          else if(pAttr->pValue == &ptacControlForceFan)
          {
            notifyApp = PTACCONTROL_FORCEFAN;
          }
          else if(pAttr->pValue == &ptacControlForceMode)
          {
            notifyApp = PTACCONTROL_FORCEMODE;
          }
        }

        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;

      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && ptacControl_AppCBs && ptacControl_AppCBs->pfnPtacControlChange )
  {
    ptacControl_AppCBs->pfnPtacControlChange( notifyApp );
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/
