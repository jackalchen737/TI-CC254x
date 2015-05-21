#include "hal_uart.h"
#include "serialInterface.h"
#include "simpleBLEperipheral.h"

/*
This is the modified version of serial interface by Jackal Chen
jackalchen737@gmail.com
*/
static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

static uint8 serialInterface_TaskID;   // Task ID for internal task/event processing
static BLEPacket_t  rxSerialPkt;
static SerialEventPacket_t txSerialPkts[TXQ_SIZE];
static uint8 txqIn = 0, txqOut = 0;

/*
TX Queue Process
in == out => empty
(in + 1) % TXQ_SIZE == out => full
*/

//return null if no free packet
static SerialEventPacket_t* getFreePacketFromTxQueue(){
    uint8 nextIn = (txqIn + 1) % TXQ_SIZE;

    if(nextIn == txqOut){
        return 0;
    }
    else{
        uint8 tempTxIn= txqIn;
        txqIn = nextIn;
        return &(txSerialPkts[tempTxIn]);
    }
}

//return null if no used packets
static SerialEventPacket_t* getUsedPacketFromTxQueue(){

    if(txqIn == txqOut){
        return 0;
    }
    else{
        uint8 tempTxOut = txqOut;
        txqOut = (txqOut + 1) % TXQ_SIZE;
        return &(txSerialPkts[tempTxOut]);
    }
}

void SerialInterface_Init( uint8 task_id )
{
    serialInterface_TaskID = task_id;

    NPI_InitTransport(cSerialPacketParser);

    //test print
    //sendSerialString("serial init\r\n", 13);
}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{

    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
        {
            SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SI_CMD_RX)
    {

        parseCmd();

        return ( events ^ SI_CMD_RX);
    }

    if ( events & SI_EVT_TX)
    {

        sendSerialEvt();

        return ( events ^ SI_EVT_TX);
    }

    // Discard unknown events
    return 0;
}

static void sendSerialResp(uint8 cmdcode, uint8 status)
{
    SerialEventPacket_t *pTxSerialPkt = getFreePacketFromTxQueue();

    if(pTxSerialPkt == 0)
        return;

    pTxSerialPkt->header.identifier = SERIAL_IDENTIFIER;
    pTxSerialPkt->header.opCode = APP_EVT_CMD_RESPONSE;
    pTxSerialPkt->cmdCode = cmdcode;
    pTxSerialPkt->status = status;
    
    osal_set_event( serialInterface_TaskID, SI_EVT_TX );
}


void sendSerialString(uint8 *str, uint8 len)
{
    SerialEventPacket_t *pTxSerialPkt = getFreePacketFromTxQueue();

    if(pTxSerialPkt == 0)
        return;

    #if 1
    pTxSerialPkt->header.identifier = SERIAL_IDENTIFIER;
    pTxSerialPkt->header.opCode = APP_EVT_LOG;
    pTxSerialPkt->datalen = len;

    if(len < 64)
    {
        osal_memcpy(pTxSerialPkt->data, str, len);
    }
    else
    {
        osal_memcpy(pTxSerialPkt->data, str, 64);
    }

    osal_set_event( serialInterface_TaskID, SI_EVT_TX );
    #else
    HalUARTWrite(HAL_UART_PORT_0, str, len);
    #endif
}


static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {
    default:
        // do nothing
        break;
    }
}

void cSerialPacketParser( uint8 port, uint8 events )
{
    (void)port;
    static npi_serial_parse_state_t  pktState = NPI_SERIAL_STATE_ID;
    uint8           done = FALSE;
    uint16          numBytes;
    static uint8    cmd_identifier;
    static uint8    cmd_opcode;
    static uint8   cmd_len;

    if (events & HAL_UART_RX_TIMEOUT)
    {
        // get the number of available bytes to process
        numBytes = NPI_RxBufLen();
        // check if there's any serial port data to process
        while ( (numBytes > 0) && (!done) )
        {
            // process serial port bytes to build the command or data packet
            switch( pktState )
            {

            case NPI_SERIAL_STATE_ID:
            {
                (void)NPI_ReadTransport((uint8 *)&cmd_identifier, 1);
                // decrement the number of available bytes
                numBytes -= 1;

                if(cmd_identifier != SERIAL_IDENTIFIER)
                {
                    // illegal packet type
                    return;
                }
                rxSerialPkt.header.identifier = cmd_identifier;
                pktState = NPI_SERIAL_STATE_OPCODE;
                break;
            }

            case NPI_SERIAL_STATE_OPCODE:
            {
                // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                (void)NPI_ReadTransport((uint8 *)&cmd_opcode, 1);
                // decrement the number of available bytes
                numBytes -= 1;

                // set next state based on the type of packet
                switch( cmd_opcode )
                {
                case APP_CMD_ADVERTISE:
                case APP_CMD_STOP_ADVERTISE:
                case APP_CMD_DATA:
                    rxSerialPkt.header.opCode = cmd_opcode;
                    pktState = NPI_SERIAL_STATE_LEN;
                    break;
                default:
                    // illegal packet type
                    return;
                }
                break;
            }

            case NPI_SERIAL_STATE_LEN: // command length
            {
                if (numBytes < 1)
                {
                    // not enough data to progress, so leave it in driver buffer
                    done = TRUE;
                    break;
                }
                // read the length
                // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                (void)NPI_ReadTransport((uint8 *)&cmd_len, 1);
                rxSerialPkt.length = cmd_len;
                // decrement the number of available bytes
                numBytes -= 1;
                pktState = NPI_SERIAL_STATE_DATA;
                break;
            }

            case NPI_SERIAL_STATE_DATA:       // command payload
            {
                // check if there is enough serial port data to finish reading the payload
                if ( numBytes < cmd_len )
                {
                    // not enough data to progress, so leave it in driver buffer
                    done = TRUE;
                    break;
                }
                else
                {
                    if(cmd_len > 0)
                    {
                        (void) NPI_ReadTransport((uint8 *)rxSerialPkt.data, cmd_len);
                    }
                    //we must handle data len = 0, process opcode only
                    pktState = NPI_SERIAL_STATE_ID;
                    done = TRUE;
                    osal_set_event( serialInterface_TaskID, SI_CMD_RX );
                    break;
                }
            }
            }

        }
    }
    else {
        return;
    }
}


void parseCmd(void)
{
    uint8 opCode =  rxSerialPkt.header.opCode;
    uint16 dur = 0;
    uint16 interval = 100;

    switch (opCode)
    {
    case APP_CMD_ADVERTISE:
    {
        if(rxSerialPkt.length != 0)
        {
            dur = BUILD_UINT16(rxSerialPkt.data[1], rxSerialPkt.data[0]) * 1000;
            interval = BUILD_UINT16(rxSerialPkt.data[3], rxSerialPkt.data[2]);
        }

        uint8 status = Application_StartAdvertise(dur, interval);
        
        sendSerialResp(opCode, status);
        
        break;
    }
    case APP_CMD_STOP_ADVERTISE:
    {
        uint8 status = Application_StopAdvertise(dur, interval);
        
        sendSerialResp(opCode, status);

        break;
    }
    case APP_CMD_DATA:
        //you can handle rxSerialPkt.data & rxSerialPkt.length here

        sendSerialResp(opCode, SUCCESS);
        break;
    }
}

void sendSerialEvt(void)
{
    SerialEventPacket_t *pTxSerialPkt = 0;
    while((pTxSerialPkt = getUsedPacketFromTxQueue()) != 0){

        uint8 opCode =  pTxSerialPkt->header.opCode;
        
        switch (opCode)
        {
    
        case APP_EVT_CMD_RESPONSE:
            if(pTxSerialPkt->status == SUCCESS)
                HalUARTWrite(HAL_UART_PORT_0, "RESP > SUCCESS\r\n", 16);
            else
                HalUARTWrite(HAL_UART_PORT_0, "RESP > FAIL\r\n", 13);
            break;
    
        case APP_EVT_LOG:
            //send string data for debug trace
            HalUARTWrite(HAL_UART_PORT_0, pTxSerialPkt->data, pTxSerialPkt->datalen);
            break;
    
        }

    }
}

