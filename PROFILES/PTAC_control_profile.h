/******************************************************************************

 @file  simple_gatt_profile.h

 @brief This file contains the Simple GATT profile definitions and prototypes
        prototypes.

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

#ifndef PTACCONTROL_H
#define PTACCONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define PTACCONTROL_SETTEMPERATURE             0  // RW uint8 - Profile Characteristic 1 value
#define PTACCONTROL_FORCEFAN                   1  // RW uint8 - Profile Characteristic 2 value
#define PTACCONTROL_FORCEHEAT                  2  // RW uint8 - Profile Characteristic 3 value
#define PTACCONTROL_FORCECOOL                  3  // RW uint8 - Profile Characteristic 4 value
#define PTACCONTROL_ACTUALTEMPERATURE          4  // RW uint8 - Profile Characteristic 4 value

// Simple Profile Service UUID
#define PTACCONTROL_SERV_UUID                  0xFFF1

// Parameter UUID
#define PTACCONTROL_SETTEMPERATURE_UUID        0xFFF1
#define PTACCONTROL_FORCEFAN_UUID              0xFFF2
#define PTACCONTROL_FORCEHEAT_UUID             0xFFF3
#define PTACCONTROL_FORCECOOL_UUID             0xFFF4
#define PTACCONTROL_ACTUALTEMPERATURE_UUID     0xFFF5

// Simple Keys Profile Services bit fields
#define PTACCONTROL_SERVICE               0x00000001

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*ptacControlChange_t)( uint8 paramID );

typedef struct
{
  ptacControlChange_t        pfnPtacControlChange;  // Called when characteristic value changes
} ptacControlCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * PtacControl_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t PtacControl_AddService( uint32 services );

/*
 * PtacControl_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t PtacControl_RegisterAppCBs( ptacControlCBs_t *appCallbacks );

/*
 * PtacControl_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t PtacControl_SetParameter( uint8 param, uint8 len, void *value );

/*
 * PtacControl_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t PtacControl_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
