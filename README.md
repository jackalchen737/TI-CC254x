MySimpleBLEPeripheralWithSerialInterface

	This is modified version of serial interface for TI CC254x.
	Original version can be found at 'http://processors.wiki.ti.com/index.php/SimpleBLEPeripheral_SerialInterface'.
	I add a circular tx message queue to preven message lost and also some command parsing for DEMO usage. 
	
	OSAL_SimpleBLEPeripheral.c
	serialInterface.c						=> Serial Lib that can be reused
	serialInterface.h						=> Serial Lib that can be reused
	simpleBLEPeripheral.c				=> Example to use Serial Lib
	simpleBLEPeripheral.h
	SimpleBLEPeripheral_Main.c

	For detail please refer to the URL below:
	http://thinkingiot.blogspot.tw/2015/05/ti-cc254x-simplebleperipheral-with.html
	
MySimpleBLEPeripheralBroadcaster

	This is modified version of Peripheral + Broadcaster of TI CC254x
	For detail please refer to the URL below:
	http://thinkingiot.blogspot.tw/2015/05/ti-cc254x-peripheral-broadcaster.html

[MyUARTBridge]
	This is modified version of UART2BLE Bridge of TI CC254x
	For detail please refer to the URL below:
	http://thinkingiot.blogspot.tw/2015/04/ti-cc254x-ble-spp-serial-port-profile-1.html
	http://thinkingiot.blogspot.tw/2015/05/ti-cc254x-ble-spp-serial-port-profile-2.html
	
