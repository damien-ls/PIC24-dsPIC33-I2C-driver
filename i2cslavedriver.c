
#include "hardwareprofile.h"
#include "i2cslavedriver.h"
#include <xc.h>
//#include <p24Exxxx.h>


// State 1 - I2C2STAT bits: S = 1, D_A = 0, R_W = 0   //Address matched
#define state_1 0b00000000
// State 2 - I2C2STAT bits: S = 1, D_A = 0, R_W = 1   //Address matched
#define state_2 0b00000100
// State 3 - I2C2STAT bits: S = 1, D_A = 1, R_W = 1	  // Master Read => Slave send data
#define state_3 0b00100100
// State 4 - I2C2STAT bits: S = 1, D_A = 1, R_W = 0   // Master Write => Slave receive data
#define state_4 0b00100000


#define State_1 0x01    // address matched write
#define State_2 0x02	// address matched read
#define State_3 0x03	// Master Read => Slave send data
#define State_4 0x04	// Master Write => Slave receive data



#define I2CSTAT_BIT_MASK  0b00100100            // Mask for I2C status bits
// bit 2 R/W: Read/Write bit Information (I2C mode only)
// bit 3 S: START bit
// bit 5 D/A: Data/Address bit 

  

#define I2C_IDLE 		 0  
#define I2C_WRITE        1      
#define I2C_READ         2    
#define I2C_ERR			0xFF


unsigned int sendCount, receieveCount, i2cCurrentBuffer;
char * i2cReadyBuffer;

typedef struct { 
        unsigned char wasRead;
		unsigned char address ;
		unsigned char addressMask ;
        unsigned char *sendBuffer;
		unsigned char *receiveBuffer[2];
        unsigned int receiveBufferSplit;
		unsigned int sendBufferSize;
		unsigned int receiveBufferSize;    
		void (*fp_dataReady)() ;
        void (*fp_readRequest)() ;
		void (*fp_dataSent)() ;
} I2CSLAVE_; 
I2CSLAVE_ slave;


void i2cInterrupt();
void i2cActions(char i2cStatus, char moduleId) ;
void i2cInitHardwareI2C1(void) ;
void i2cInitHardwareI2C2(void) ;


int i2cDoEvents(char i2cModule)
{
    char stopBit;

    if (i2cModule == 1) stopBit = I2C1STATbits.P;
    else stopBit = I2C2STATbits.P;
    
    if ((stopBit == 1) && (slave.wasRead == 0))
    {
        i2cReadyBuffer = slave.receiveBuffer[i2cCurrentBuffer];      
        i2cCurrentBuffer++;
        if (i2cCurrentBuffer >= slave.receiveBufferSplit) i2cCurrentBuffer = 0;
                
        (*slave.fp_dataReady)( i2cReadyBuffer ); // call function pointer back to application
        slave.wasRead = 1;
        
        return 1;
    }
    
    return 0;
}

void i2cSetSlaveSendBuffer(unsigned char * buffer, int size) 
{
    slave.sendBuffer = buffer;
    slave.sendBufferSize = size;
    sendCount = 0;
}

/*
void i2cSetSlaveReceiveBuffer(unsigned char * buffer, int size) 
{
    slave.receiveBuffer = buffer;
    slave.receiveBufferSize = size;
}

unsigned char * i2cReceiveBuffer() 
{
    return slave.receiveBuffer;
}

unsigned char * i2cSendBuffer() 
{
    return slave.sendBuffer;
}
*/
void i2cSlaveInit(unsigned char address,  unsigned char maskAddress, void (*fp_readRequest), 
                  void (*fp_dataReady)(), void (*fp_dataSent)(),     unsigned char * receiveBuffer,
                  int receiveBufferSize,  int receiveBufferSplit,    unsigned char * sendBuffer, 
                  int sendBufferSize, char i2cModule) 
{
    int i;
    
    slave.address = address;
    slave.addressMask = maskAddress;
    slave.fp_dataReady = fp_dataReady;
    slave.fp_dataSent = fp_dataSent;
    slave.fp_readRequest = fp_readRequest;
    slave.sendBuffer = sendBuffer;
    slave.receiveBufferSize = receiveBufferSize;
    slave.receiveBufferSplit = receiveBufferSplit;
    slave.sendBufferSize = sendBufferSize;
    slave.wasRead = 0;
    

    int singleBufSize = receiveBufferSize / receiveBufferSplit;
    
    for (i=0; i<receiveBufferSplit; i++)
            slave.receiveBuffer[i] = &receiveBuffer[singleBufSize * i];
    
    i2cCurrentBuffer = 0;
    i2cReadyBuffer = slave.receiveBuffer[0];
    
    if (i2cModule == 1) i2cInitHardwareI2C1();
    else if (i2cModule == 2) i2cInitHardwareI2C2();
}

void i2cInitHardwareI2C1(void) 
{
    I2C1CONbits.I2CEN = 0;
    
    I2C1ADD = slave.address;
    I2C1MSK = slave.addressMask;
    
    //I2C1CONHbits.PCIE = 1; // this device does not implement the stop bit interrupt
    
    I2C1CONbits.IPMIEN = 0; // enable address masking
    I2C1CONbits.A10M = 0; // Address mode 7 bits
    I2C1CONbits.STREN = 1; // clock stretching
    I2C1CONbits.GCEN = 1; // general call address interrupt enable

    _SI2C1IF = 0;
    _SI2C1IP = 5;
    _SI2C1IE = 1;
    
    I2C1CONbits.I2CEN = 1;
}


void i2cInitHardwareI2C2(void) 
{
    I2C2CONbits.I2CEN = 0;
    
    I2C2ADD = slave.address;
    I2C2MSK = slave.addressMask;
    
    // to test if this below is required or not. Seem not possible to reach I2C2CONH despite it's on the datasheet.
    //I2C2CON = 0;
   // I2C2CON = (unsigned long) 1 << (6+16); // set PCIE  Stop Condition Interrupt Enable bit

    I2C2CONbits.IPMIEN = 0; // enable address masking
    I2C2CONbits.A10M = 0; // Address mode 7 bits
    I2C2CONbits.STREN = 1; // clock stretching
    I2C2CONbits.GCEN = 1; // general call address interrupt enable
           
            
    _SI2C2IF = 0;
    _SI2C2IP = 5;
    _SI2C2IE = 1;
    
    I2C2CONbits.I2CEN = 1;
}



void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void) 
{
    _SI2C1IF = 0;
    char status = (I2C1STAT & I2CSTAT_BIT_MASK);
    i2cActions( status, 1);
}
void __attribute__((interrupt, no_auto_psv)) _SI2C2Interrupt(void) 
{
    _SI2C2IF = 0;
    char status = (I2C2STAT & I2CSTAT_BIT_MASK);
    i2cActions( status,  2);
}


void i2cActions(char i2cStatus, char moduleId) 
{
    static unsigned char latestByte=0; 
    char i2c_state;
    unsigned char dummy; //used for dummy read

    if ((i2cStatus ^ state_1) == 0)
        i2c_state = 0x01;
    else if ((i2cStatus ^ state_2) == 0)
        i2c_state = 0x02;
    else if ((i2cStatus ^ state_3) == 0)
        i2c_state = 0x03; // 
    else if ((i2cStatus ^ state_4) == 0)
        i2c_state = 0x04; // 
    else
        i2c_state = 0;

    switch (i2c_state) { //state 1:	Address matched
        case State_1: // address matched write
            if (moduleId == 1) {
                dummy = I2C1RCV; //dummy read
                I2C1CONbits.SCLREL = 1; //Release SCL1 line
            } else {
                dummy = I2C2RCV; //dummy read
                I2C2CONbits.SCLREL = 1; //Release SCL1 line
            }
            
            slave.wasRead = 0;
            receieveCount = 0;
            sendCount = 0;
            break;

        case State_2: // address matched read
            if (moduleId == 1) {
                dummy = I2C1RCV; //dummy read
            } else {
                dummy = I2C2RCV; //dummy read
            }
           //release the line in state 3
            
            receieveCount = 0;
            sendCount = 0;

            (*slave.fp_readRequest)(latestByte); // call function pointer back to application, this will set the pointer to the correct buffer
            //break; NOT BREAK, DIRECTLY SEND THE FIRST BYTE IN STATE 3


        case State_3: // Master Read => Slave send data
            if (moduleId == 1) {
                I2C1TRN = slave.sendBuffer[sendCount]; //Read data from RAM & send data to I2C master device
                I2C1CONbits.SCLREL = 1; //Release SCL1 line
            } else {
                I2C2TRN = slave.sendBuffer[sendCount]; //Read data from RAM & send data to I2C master device
                I2C2CONbits.SCLREL = 1; //Release SCL1 line
            }

            if (sendCount < slave.sendBufferSize) sendCount++;
            else (*slave.fp_dataSent)(); // call function pointer back to application

            break;

        case State_4: // Master Write => Slave receive data
            if (moduleId == 1) {
                latestByte = I2C1RCV;
                I2C1CONbits.SCLREL = 1; //Release SCL1 line
            } else {
                latestByte = I2C2RCV;
                I2C2CONbits.SCLREL = 1; //Release SCL1 line
            }

            slave.receiveBuffer[i2cCurrentBuffer][receieveCount] = latestByte;
            if (receieveCount < slave.receiveBufferSize) receieveCount++;
            
            break;
            
            
        default:
            if (moduleId == 1) 
            {
               I2C1CONbits.SCLREL = 1; //Release SCL1 line 
               i2cInitHardwareI2C1(); 
            } else {
               I2C2CONbits.SCLREL = 1; //Release SCL1 line
                i2cInitHardwareI2C2(); 
            }
            break;

    } // End Switch
    
}



