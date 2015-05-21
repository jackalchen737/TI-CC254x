[MySimpleBLEPeripheralWithSerialInterface]

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
	XXXXX
	
...