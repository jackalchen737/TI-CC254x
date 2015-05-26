#include "hal_uart.h"
#include "OSAL.h"
#include "npi.h"

/*
This is the modified version of serial interface by Jackal Chen
jackalchen737@gmail.com
*/


#define SI_CMD_RX                                 0x0001
#define SI_EVT_TX                                 0x0002

#define MAX_PKT_SIZE 128

//cmd packet prefix
#define SERIAL_IDENTIFIER       0x77

//rx opcode
#define APP_CMD_ADVERTISE       0x01
#define APP_CMD_STOP_ADVERTISE  0x02
#define APP_CMD_DATA            0x03
#ifdef PLUS_OBSERVER
#define APP_CMD_START_OBSERVING 0x04
#define APP_CMD_LIST_OBSERVING  0x05
#endif


//tx opcode
#define	APP_EVT_CMD_RESPONSE	0x81
#define APP_EVT_LOG             0x82

//C Queue
#define TXQ_SIZE    10

typedef struct _BLEPacketHeader
{
    uint8 identifier;
    uint8 opCode;
} BLEPacketHeader_t;


typedef struct _BLEPacket
{
    BLEPacketHeader_t header;
// payload
    uint8 length;  // opCode specific,
    uint8 data[10];  // command or event specific data, 10 Bytes currently
} BLEPacket_t;

typedef struct _SerialEventPacket
{
    BLEPacketHeader_t header;
// used in RESP
    uint8 cmdCode; // Command Code
    uint8 status;  // Status
//used in DATA
    uint8 datalen;  //data length
    uint8 data[64]; //max 64 bytes
} SerialEventPacket_t;


/* States for CRC parser */
typedef enum
{
    NPI_SERIAL_STATE_ID,
    NPI_SERIAL_STATE_OPCODE,
    NPI_SERIAL_STATE_LEN,
    NPI_SERIAL_STATE_DATA,
} npi_serial_parse_state_t;

void cSerialPacketParser( uint8 port, uint8 events );
void parseCmd(void);
void sendSerialEvt(void);
extern void sendSerialString(uint8 *str, uint8 len);
extern void SerialInterface_Init( uint8 task_id );
extern uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events );

