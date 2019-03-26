
# PIC24-dsPIC33-I2C-driver
Simple to use PIC24 / dsPIC33 I2C slave driver

## A simple I2C driver for PIC24 and dsPIC33
As it might be useful for some, this slave driver is enclosed and easy to use.

# Usage:
## Initialisation

    #define I2CBUFFERSIZE 512
    
    void i2cReadRequest(unsigned char command);
    void i2cDataReady(char * buffer);
    void i2cDataSent();
    
    unsigned char i2cReceiveBuffer[I2CBUFFERSIZE];
    unsigned char i2cReceiveBuffer[I2CBUFFERSIZE];

    void initApplication()
	{
    
	    i2cSlaveInit(   address , 
	                    mask , 
	                    (void *) i2cReadRequest, // callback for read request
	                    (void *) i2cDataReady, // callback for write request
	                    (void *) i2cDataSent, // callback for datasent
	                    i2cReceiveBuffer, 
	                    sizeof (i2cReceiveBuffer), 
	                    2, 
	                    i2cSendBuffer, 
	                    sizeof (i2cSendBuffer), 
	                    1 // I2C driver (I2C1 or I2C2)
	            );
	}

## Events handling
Call from the main loop:

    void main ()
    {
	    while (1) {
		    i2cDoEvents(1); // I2C driver (I2C1 or I2C2)
	    }
    }

## Callbacks

Read Request:

    char someData[32];
    void i2cReadRequest(unsigned char command)
    {    
        switch (command)
        {
            case SOMETHING:
                i2cReply(someData);
                break;
    
            default:
                break;
        }
    }

Write Request:

    void i2cDataReady(char * buffer)
    {
        unsigned char * buf = (unsigned char * ) &buffer[1];
    
        switch (buffer[0])
        {
            case SOMETHING:
                doSomething(buf);
                break;
        }
    }

Data sent:

    void i2cDataSent()
    {
        toggleLed();
    }
