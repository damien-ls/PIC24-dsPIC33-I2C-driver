
#if defined (__dsPIC33E__)
	#include <p33exxxx.h>
#elif defined (__dsPIC24E__)
	#include <p24exxxx.h>
#endif

#define i2cReply(x) i2cSetSlaveSendBuffer((unsigned char *) &x, sizeof(x))

int i2cDoEvents(char i2cModule);
void i2cSlaveInit(unsigned char address, unsigned char maskAddress, void (*fp_readRequest), void (*fp_dataReady)(), void (*fp_dataSent)(), unsigned char * receiveBuffer, int receiveBufferSize, int receiveBufferSplit, unsigned char * sendBuffer, int sendBufferSize, char i2cModule);
// deprecated - not used? void i2cSetSlaveReceiveBuffer(unsigned char * receive_buffer, int size) ;
void i2cSetSlaveSendBuffer(unsigned char * send_buffer, int size) ;
