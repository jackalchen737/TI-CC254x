#include "hal_uart.h"
#include "OSAL.h"
#include "npi.h"

#define MAX_PKT_SIZE    128
#define RX_BUFF_SIZE    500

//===================================================

/* States for CRC parser */
typedef enum {
  SERIAL_STATE_START = 0,
  SERIAL_STATE_TYPE,
  SERIAL_STATE_LEN,
  SERIAL_STATE_DATA,
  SERIAL_STATE_COMPLETE //received complete serial message
} serial_state_t;

void cSerialPacketParser( uint8 port, uint8 events );
void parseCmd(void);
void sendSerialEvt(void);


/*******************************************************************************
 * MACROS
 */

//global

extern uint16 serialBufferOffset;
extern uint8 serialBuffer[RX_BUFF_SIZE];

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SerialInterface_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events );

extern uint8 sendAckMessage(uint8 bytes_sent);

extern uint8 sendDataToHost(uint8* data, uint8 len);  
  
extern uint16 circular_add(uint16 x, uint16 y);

extern uint16 circular_diff(uint16 offset, uint16 tail);