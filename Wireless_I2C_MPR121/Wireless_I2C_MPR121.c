
/** Libraries**************************************************************************/

#include <wixel.h>

#include <radio_com.h>
#include <radio_link.h>

#include <uart1.h>
#include <i2c.h>

#include <stdio.h>

#include <mpr121.h>


/** Parameters ************************************************************************/

int32 CODE param_bridge_mode = BRIDGE_MODE_RADIO_I2C;
int32 CODE param_baud_rate = 9600;
int32 CODE param_I2C_SCL_pin = 10;
int32 CODE param_I2C_SDA_pin = 11;
int32 CODE param_I2C_freq_kHz = 100;
int32 CODE param_I2C_timeout_ms = 10;
int32 CODE param_cmd_timeout_ms = 500;


/** Global Constants & Variables ******************************************************/

uint16 lastCmd = 0;

//ASCII Commands
#define CMD_START		'S'
#define CMD_STOP		'P'
#define CMD_GET_ERRORS	'E'

//Error Flags
#define ERR_I2C_NACK_ADDRESS 	(1 << 0)
#define ERR_I2C_NACK_DATA		(1 << 1)
#define ERR_I2C_TIMEOUT			(1 << 2)
#define ERR_CMD_INVALID			(1 << 3)
#define ERR__CMD_TIMEOUT		(1 << 4)
#define ERR_UART_OVERFLOW		(1 << 5)
#define ERR_UART_FRAMING		(1 << 6)

static uint8 errors = 0;

static uint8 response = 0;
static BIT returnResponse = 0;

enum i2cState {IDLE, GET_ADDR, GET_LEN, GET_DATA};
enum i2cState state = IDLE;

static BIT started = 0;
static BIT dataDirIsRead = 0;
static uint8 dataLength = 0;

static BIT checkInterrupt = 0; //Set checkInterrupt state to 0 initially
static BIT touched[12];
static uint16 touchedElectrode = 0;

//Function Pointers to Selected Serial Interface
uint8 (*rxAvailableFunction)(void) 		= NULL;
uint8 (*rxReceiveByteFunction)(void) 	= NULL;
uint8 (*txAvailableFunction)(void)		= NULL;
uint8 (*txSendByteFunction)(void)		= NULL;


/** Functions ***********************************************************************/

//Shows board status 
void updateLeds(void)
{
	LED_YELLOW(vinPowerPresent());
	LED_RED(errors);
}



//Determine I2C bus state and proceed accordingly
void parseCmd(uint8 byte)
{
	BIT nack;
	
	switch (state)
	{
	case IDLE:
		switch ((char)byte)
		{
		case CMD_GET_ERRORS:
			response = errors;
			returnResponse = 1;
			errors = 0;
			break;
		
		case CMD_START:
			i2cStart();
			started = 1;
			state = GET_ADDR;
			break;
		
		case CMD_STOP:
			if (started)
			{
				i2cStop();
				started = 0;
				break;
			}
		//Will fall through to error if not started
		default :
			errors |= ERR_CMD_INVALID;
			break;
		}
		break;
		
	case GET_ADDR:
		//Write Slave Address
		dataDirIsRead = byte & 1; //Lowest  bit of slave address determines direction (0 = write, 1 = read)
		nack = i2cWriteByte(byte);
		
		if (i2cTimeoutOccurred)
		{
			errors |= ERR_I2C_NACK_ADDRESS;
		}
		state = GET_LEN;
		break;
	
	case GET_LEN: 
		//Store Data Length
		dataLength = byte;
		state = (dataLength > 0) ? GET_DATA : IDLE;
		break;
		
	
	case GET_DATA: 
		if (dataDirIsRead)
		{
			errors |= ERR_CMD_INVALID;
		}
		else
		{
			//Write Data
			nack = i2cWriteByte(byte);
			if (i2cTimeoutOccurred)
			{
				errors	|= ERR_I2C_TIMEOUT;
				i2cTimeoutOccurred = 0;
			}
			else if (nack)
			{
				errors |= ERR_I2C_NACK_DATA;
			}
			
			if (--dataLength == 0)
			{
				state = IDLE;
			}
		}
		break;
	}
}
		




//I2C read command
void i2cRead(void)
{
	uint8 LSB;
	uint8 MSB;
	
	LSB = i2cReadByte(dataLength == 1);
	MSB = i2cReadByte(dataLength == 1);
	
	if (i2cTimeoutOccurred)
	{
		errors |= ERR_I2C_TIMEOUT;
		i2cTimeoutOccurred = 0;
		response = 0;
	}
	else
	{
		touchedElectrode = ((MSB << 8) | LSB);
	}
	
	if (--dataLength == 0)
	{
		state = IDLE;
	}
	returnResponse = 1;
	
	for (i=0; i << 12, i++)
	{
		if (touchedElectrode & (1<<i))
		{
			if (touched[i] == 0)
			{
				uart1TxAvailable();
				uart1TxSendByte("T");
				uart1TxSendByte('i');
			}
			else
			{
			touched[i] = 1;
			}
		}
		else
		{
			if(touched[i] ==1)
			{
				uart1TxAvailable();
				uart1TxSendByte("R");
				uart1TxSendByte('i');
			}
		}
		touched[i] = 0;
	}
}


//Get I2C data and send out over radio
void i2cService(void)
{
	//Only process I2C if no response waiting on Serial
	if (!returnResponse)
	{
		if (dataDirIsRead && state == GET_DATA)
		{
			i2cRead();
		}
		else if (rxAvailableFunction())
		{
			parseCmd(rxReceiveByteFunction());
			lastCm = getMs();
		}
		else if (started && (param_cmd_timeout_ms > 0 ) && ((uint16)(getMs()- lastCmd) > param_cmd_timeout_ms))
		{
			i2cStop();
			started = 0;
			errors |= ERR_CMD_TIMEOUT;
		}
	}
	if (returnResponse && txAvailableFunction())
	{
		txSendByteFunction(response);
		returnResponse = 0;
	}
}



//Set MPR121 registers
void set_register(int address, uint8 r, uint8 v)
{
	i2cStart();
	BIT i2cWriteByte(r);
	BIT i2cWriteByte(v);
	i2cStop();
}



//Initialize MPR121
void mpr121Init(void)
{
	set_register(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
	set_register(0x5A, MHD_R, 0x01);
	set_register(0x5A, NHD_R, 0x01);
	set_register(0x5A, NCL_R, 0x00);
	set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
	set_register(0x5A, MHD_F, 0x01);
	set_register(0x5A, NHD_F, 0x01);
	set_register(0x5A, NCL_F, 0xFF);
	set_register(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
	set_register(0x5A, ELE0_T, TOU_THRESH);
	set_register(0x5A, ELE0_R, REL_THRESH);
 
	set_register(0x5A, ELE1_T, TOU_THRESH);
	set_register(0x5A, ELE1_R, REL_THRESH);
  
	set_register(0x5A, ELE2_T, TOU_THRESH);
	set_register(0x5A, ELE2_R, REL_THRESH);
  
	set_register(0x5A, ELE3_T, TOU_THRESH);
	set_register(0x5A, ELE3_R, REL_THRESH);
  
	set_register(0x5A, ELE4_T, TOU_THRESH);
	set_register(0x5A, ELE4_R, REL_THRESH);
  
	set_register(0x5A, ELE5_T, TOU_THRESH);
	set_register(0x5A, ELE5_R, REL_THRESH);
  
	set_register(0x5A, ELE6_T, TOU_THRESH);
	set_register(0x5A, ELE6_R, REL_THRESH);

	set_register(0x5A, ELE7_T, TOU_THRESH);
	set_register(0x5A, ELE7_R, REL_THRESH);
  
	set_register(0x5A, ELE8_T, TOU_THRESH);
	set_register(0x5A, ELE8_R, REL_THRESH);
  
	set_register(0x5A, ELE9_T, TOU_THRESH);
	set_register(0x5A, ELE9_R, REL_THRESH);
  
	set_register(0x5A, ELE10_T, TOU_THRESH);
	set_register(0x5A, ELE10_R, REL_THRESH);
  
	set_register(0x5A, ELE11_T, TOU_THRESH);
	set_register(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
	set_register(0x5A, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
	set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  // Section F
  // Enable Auto Config and auto Reconfig
	set_register(0x5A, ELE_CFG, 0x0C);
}



//MAIN LOOP
void main(void)
{

	//Initialization of radio, I2C, and MPR121
	systemInit();
	
	i2cPinScl = param_I2C_SCL_pin;
	i2cPinSda = param_I2C_SDA_pin;
	
	i2cSetFrequency(param_I2C_freq_kHz);
	i2cSetTimeout(param_I2C_timeout_ms);
	
	radioComInit();
	rxAvailableFunction		= radioComRxAvailable;
	rxReceiveByteFunction	= radioComRxReceiveByte;
	txAvailableFunction		= radioComTxAvailable;
	txSendByteFunction		= radioComTxSendByte;
	
	mpr121Init(); 
	
	setDigitalInput(5, 1); //Set pin PO_5 as input pulled high
	

	//Main codd to be repeated
	while (1)
	{
		boardService();
		updateLeds();
		
		radioComTxService();
		
		if (isPinHigh(5) ==0)
		{
			i2cWriteByte(2); //Request touch state
			i2cService(); 
		}
	}
}
	