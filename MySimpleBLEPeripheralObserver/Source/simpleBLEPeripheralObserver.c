/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined ( PLUS_BROADCASTER )
#include "peripheralBroadcaster.h"
#else
#include "peripheralObserverProfile.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheralObserver.h"

#if defined FEATURE_OAD
#include "oad.h"
#include "oad_target.h"
#endif

#if defined (SERIAL_INTERFACE)
#include "serialinterface.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
#define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

#ifdef PLUS_OBSERVER
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x14,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x53,   // 'S'
    0x69,   // 'i'
    0x6d,   // 'm'
    0x70,   // 'p'
    0x6c,   // 'l'
    0x65,   // 'e'
    0x42,   // 'B'
    0x4c,   // 'L'
    0x45,   // 'E'
    0x50,   // 'P'
    0x65,   // 'e'
    0x72,   // 'r'
    0x69,   // 'i'
    0x70,   // 'p'
    0x68,   // 'h'
    0x65,   // 'e'
    0x72,   // 'r'
    0x61,   // 'a'
    0x6c,   // 'l'

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

    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,   // length of this data
    GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
    LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
    HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

#ifdef PLUS_OBSERVER

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

#ifdef PLUS_OBSERVER
static void observerEventCB( observerRoleEvent_t *pEvent );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

#ifdef PLUS_OBSERVER
// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    NULL,                            // When a valid RSSI is read from controller (not used by application)
    observerEventCB
};
#else
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};

#endif

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
    NULL,                     // Passcode callback (not used by application)
    NULL                      // Pairing / Bonding state Callback (not used by application)
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
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void SimpleBLEPeripheral_Init( uint8 task_id )
{
    simpleBLEPeripheral_TaskID = task_id;

    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

    // Setup the GAP Peripheral Role Profile
    {
#if defined( CC2540_MINIDK )
        // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
        uint8 initial_advertising_enable = FALSE;
#else
        // For other hardware platforms, device starts advertising upon initialization
        uint8 initial_advertising_enable = TRUE;
#endif

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

    // Setup the GAP Bond Manager
    {
        uint32 passkey = 0; // passkey "000000"
        uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8 mitm = TRUE;
        uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8 bonding = TRUE;
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
    }

    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
    VOID OADTarget_AddService();                    // OAD Profile
#endif

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

    // Register for all key events - This app will handle all key events
    RegisterForKeys( simpleBLEPeripheral_TaskID );

#ifdef PLUS_OBSERVER

    // Setup Observer Profile
    {
        uint8 scanRes = DEFAULT_MAX_SCAN_RES;
        GAPRole_SetParameter( GAPOBSERVERROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
    }

    // Setup GAP
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
    GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

#endif

    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

    // Enable clock divide on halt
    // This reduces active current while radio is active and CC254x MCU
    // is halted
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

    // Setup a delayed profile startup
    osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
        {
            simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

        // Start Bond Manager
        VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

        // Set timer for first periodic event
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

        return ( events ^ SBP_START_DEVICE_EVT );
    }

    if ( events & SBP_PERIODIC_EVT )
    {
        // Restart timer
        if ( SBP_PERIODIC_EVT_PERIOD )
        {
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
        }

        // Perform periodic application task
        performPeriodicTask();

        return (events ^ SBP_PERIODIC_EVT);
    }

#if defined ( PLUS_BROADCASTER )
    if ( events & SBP_ADV_IN_CONNECTION_EVT )
    {
        uint8 turnOnAdv = TRUE;
        // Turn on advertising while in a connection
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

        return (events ^ SBP_ADV_IN_CONNECTION_EVT);
    }
#endif // PLUS_BROADCASTER

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
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
    case GAPROLE_STARTED:
    {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

#if defined (SERIAL_INTERFACE)
        sendSerialString( "State > DevInit\r\n", 17 );
#endif
    }
    break;

    case GAPROLE_ADVERTISING:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "State > Advertising...\r\n", 24 );
#endif
    }
    break;

    case GAPROLE_CONNECTED:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "State > Connected\r\n", 19 );
#endif
    }
    break;

    case GAPROLE_WAITING:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "State > Waiting\r\n", 17 );
#endif
    }
    break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "State > Time Out\r\n", 18 );
#endif
    }
    break;

    case GAPROLE_ERROR:
    {
    }
    break;

    default:
    {
    }
    break;

    }

    gapProfileState = newState;


    VOID gapProfileState;     // added to prevent compiler warning with
    // "CC2540 Slave" configurations



}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
    uint8 valueToCopy;
    uint8 stat;

    // Call to retrieve the value of the third characteristic in the profile
    stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

    if( stat == SUCCESS )
    {
        /*
         * Call to set that value of the fourth characteristic in the profile. Note
         * that if notifications of the fourth characteristic have been enabled by
         * a GATT client device, then a notification will be sent every time this
         * function is called.
         */
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
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
    uint8 newValue;

    switch( paramID )
    {
    case SIMPLEPROFILE_CHAR1:
        SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

        break;

    case SIMPLEPROFILE_CHAR3:
        SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

        break;

    default:
        // should not reach here!
        break;
    }
}

uint8 Application_StartAdvertise(uint16 duration, uint16 interval)
{
    if( gapProfileState != GAPROLE_CONNECTED )
    {
        uint8 astatus;

        // toggle GAP advertisement status
        GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &astatus );
        if (astatus == FALSE)
        {
            //Set fast advertising interval for user-initiated connections
            GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, interval );
            GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, interval );
            GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, duration );
            astatus = TRUE;
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &astatus );
            return SUCCESS;
        }
    }
    return FAILURE;
}

uint8 Application_StopAdvertise()
{

    if( gapProfileState != GAPROLE_CONNECTED )
    {
        uint8 astatus;

        // toggle GAP advertisement status
        GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &astatus );
        if (astatus == TRUE)
        {
            astatus = FALSE;
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &astatus );
            return SUCCESS;
        }
    }
    return FAILURE;
}

#ifdef PLUS_OBSERVER

static char *bdAddr2Str( uint8 *pAddr )
{
    uint8       i;
    char        hex[] = "0123456789ABCDEF";
    static char str[B_ADDR_STR_LEN];
    char        *pStr = str;

    *pStr++ = '0';
    *pStr++ = 'x';

    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for ( i = B_ADDR_LEN; i > 0; i-- )
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }

    *pStr = 0;

    return str;
}


void startObserving()
{
#if defined (SERIAL_INTERFACE)
    sendSerialString( "Start Observing\r\n", 17);
#endif
    GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                    DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                    DEFAULT_DISCOVERY_WHITE_LIST );
}

void listObserving()
{
    // Display discovery results
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
        // Increment index of current result (with wraparound)
        simpleBLEScanIdx++;
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
            simpleBLEScanIdx = 0;
        }

#if defined (SERIAL_INTERFACE)
        sendSerialString( "Device: ", 8);
        sendSerialString( (uint8*)bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ), 15);
        sendSerialString( "\r\n", 2);
#endif
    }

}


static void observerEventCB( observerRoleEvent_t *pEvent )
{
    switch ( pEvent->gap.opcode )
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "GAP_DEVICE_INIT_DONE_EVENT\r\n", 28);
#endif
    }
    break;

    case GAP_DEVICE_INFO_EVENT:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "GAP_DEVICE_INFO_EVENT\r\n", 23);
#endif
        simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
    }
    break;

    case GAP_DEVICE_DISCOVERY_EVENT:
    {
#if defined (SERIAL_INTERFACE)
        sendSerialString( "GAP_DEVICE_DISCOVERY_EVENT\r\n", 28);
#endif
        // discovery complete
        simpleBLEScanning = FALSE;

        // Copy results
        simpleBLEScanRes = pEvent->discCmpl.numDevs;
        osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                     (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );

        if ( simpleBLEScanRes > 0 )
        {
#if defined (SERIAL_INTERFACE)
            sendSerialString( "BLE devices found\r\n", 19);
#endif
        }

        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;
    }
    break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
    uint8 i;

    // If result count not at max
    if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
    {
        // Check if device is already in scan results
        for ( i = 0; i < simpleBLEScanRes; i++ )
        {
            if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
            {
                return;
            }
        }

        // Add addr to scan result list
        osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
        simpleBLEDevList[simpleBLEScanRes].addrType = addrType;

        // Increment scan result count
        simpleBLEScanRes++;
    }
}

#endif




/*********************************************************************
*********************************************************************/
