/**************************************************************************************************
  Filename:       BLE_Bridge.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the BLE_Bridge sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_uart.h"
#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "BLE_Bridge.h"

#include "serialInterface.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_SEND_EVT_PERIOD                       7

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          200

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8 serialInterface_TaskID;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void sendTestData(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 BLE_Bridge_TaskID;   // Task ID for internal task/event processing

static bool connected_flag = FALSE;  //whether to try to send data or not from SerialBuffer

static uint16 buffer_tail = 0;  //last data byte sent from SerialBuffer

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x09,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'U',
    'A',
    'R',
    'T',
    '2',
    'B',
    'L',
    'E',
    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "UART2BLE";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void BLE_Bridge_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static uint8 sendData(uint16 diff);
static void simpleProfileChangeCB( uint8 paramID );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t BLE_Bridge_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
    simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BLE_Bridge_Init
 *
 * @brief   Initialization function for the BLE_Bridge App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void BLE_Bridge_Init( uint8 task_id )
{
    BLE_Bridge_TaskID = task_id;

    // Setup the GAP Peripheral Role Profile
    {
        uint8 initial_advertising_enable = TRUE;

        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;

        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }

    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

    // Set advertising interval
    {
        uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }

    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile

    // Setup the SimpleProfile Characteristic Values
    {
        uint8 charValue1 = 1;
        uint8 charValue2 = 2;
        uint8 charValue3 = 3;
        uint8 charValue4 = 4;
        uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    }

    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

    //disable halt during RF (needed for UART / SPI)
    HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);

    // Setup a delayed profile startup
    osal_set_event( BLE_Bridge_TaskID, SBP_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      BLE_Bridge_ProcessEvent
 *
 * @brief   SBLE_Bridge Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BLE_Bridge_ProcessEvent( uint8 task_id, uint16 events )
{

    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( BLE_Bridge_TaskID )) != NULL )
        {
            BLE_Bridge_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &BLE_Bridge_PeripheralCBs );

        // Start timer for polling to see if there is data to send from SerialBuffer
        osal_start_timerEx( BLE_Bridge_TaskID, SBP_SEND_EVT, SBP_SEND_EVT_PERIOD );

        return ( events ^ SBP_START_DEVICE_EVT );
    }

    if ( events & SBP_SEND_EVT )
    {
        // Restart timer
        if ( SBP_SEND_EVT_PERIOD )
        {
            osal_start_timerEx( BLE_Bridge_TaskID, SBP_SEND_EVT, SBP_SEND_EVT_PERIOD );
        }

        // Send data if not caught up and connected
        if ((buffer_tail != serialBufferOffset) && (connected_flag == TRUE))
        {
            //calculate how many bytes can be sent
            uint16 diff = circular_diff(serialBufferOffset, buffer_tail);
            //send data and update tail
            uint8 bytes_sent = sendData(diff);
            buffer_tail = circular_add(buffer_tail,bytes_sent);
            //if we sent anything over-the-air, let host MCU know it can send more bytes
            if (bytes_sent)
            {
                //keep trying to send ACK until it is a success. this may not be the desirable approach
                while(sendAckMessage(bytes_sent));
            }
        }
        return (events ^ SBP_SEND_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      BLE_Bridge_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BLE_Bridge_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {
    default:
        // do nothing
        break;
    }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
    switch ( newState )
    {
    case GAPROLE_ADVERTISING:
    {
        connected_flag = FALSE;
    }
    break;

    case GAPROLE_CONNECTED:
    {
        connected_flag = TRUE;
    }
    break;

    default:
        break;

    }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
    //max size is 20 in BLE msg
    uint8 data[19];
    uint8 len;

    switch( paramID )
    {
    case SIMPLEPROFILE_CHAR3:
        SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &data );
        len = data[0];
        //keep trying to send data until it is a success. this may not be the desirable approach
        while (sendDataToHost(&data[1], len));
        break;

    default:
        // do nothing
        break;
    }
}

/*********************************************************************
 * @fn      sendData
 *
 * @brief   parse available serial data and send max possible amount over the air
 *
 * @param   diff - how many bytes can be sent
 *
 * @return  none
 */

static uint8 sendData(uint16 diff)
{
    //can send max 4 packets per connection interval
    uint8 packets_sent = 0;
    //ensure queue of notification is successful
    bool send_error = FALSE;
    //return value to update tail and send ack to msp
    uint8 bytes_sent = 0;

    attHandleValueNoti_t noti;
    //dummy handle
    noti.handle = 0x2E;

    //counter
    uint8 i;

    while ((packets_sent < 4) &&  (diff >= 20) && (send_error == FALSE))
    {
        //send 20 bytes
        noti.len = 20;
        for (i = 0; i < 20; i++)
        {
            noti.value[i] = serialBuffer[circular_add(buffer_tail , bytes_sent+i)];
        }
        //connection handle currently hardcoded
        if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
        {
            bytes_sent += 20;
            diff -= 20;
            packets_sent++;
        }
        else
        {
            send_error = TRUE;
        }
    }
    //send remaining bytes
    if ((packets_sent < 4) && (diff > 0) && (send_error == FALSE))
    {
        noti.len = diff;
        for (i = 0; i < diff; i++)
        {
            noti.value[i] = serialBuffer[circular_add(buffer_tail, bytes_sent + i)];
        }
        if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
        {
            bytes_sent += i;
            diff -= i;//amount of data sent
        }
        else
        {
            send_error = TRUE;
        }
    }
    return bytes_sent;
}