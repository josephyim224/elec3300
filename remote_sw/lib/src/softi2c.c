/***************************************************************************************************
                                    ExploreEmbedded Copyright Notice
****************************************************************************************************
   File:   i2c.h
   Version: 15.0
   Author: ExploreEmbedded
   Website: http://www.exploreembedded.com/wiki
   Description: Contains the library routines for I2C read,write operation
  This code has been developed and tested on ExploreEmbedded boards.
  We strongly believe that the library works on any of development boards for respective controllers.
  Check this link http://www.exploreembedded.com/wiki for awesome tutorials on 8051,PIC,AVR,ARM,Robotics,RTOS,IOT.
  ExploreEmbedded invests substantial time and effort developing open source HW and SW tools, to support consider buying the ExploreEmbedded boards.

  The ExploreEmbedded libraries and examples are licensed under the terms of the new-bsd license(two-clause bsd license).
  See also: http://www.opensource.org/licenses/bsd-license.php
  EXPLOREEMBEDDED DISCLAIMS ANY KIND OF HARDWARE FAILURE RESULTING OUT OF USAGE OF LIBRARIES, DIRECTLY OR
  INDIRECTLY. FILES MAY BE SUBJECT TO CHANGE WITHOUT PRIOR NOTICE. THE REVISION HISTORY CONTAINS THE INFORMATION
  RELATED TO UPDATES.

  Permission to use, copy, modify, and distribute this software and its documentation for any purpose
  and without fee is hereby granted, provided that this copyright notices appear in all copies
  and that both those copyright notices and this permission notice appear in supporting documentation.
***************************************************************************************************/

/***************************************************************************************************
                             Revision History
****************************************************************************************************
  15.0: Initial version
***************************************************************************************************/

/*
 * This soft i2c implementation is from ExploreEmbedded.
 * Modified from generic 8051 to stm8s003.
 */

#include "softi2c.h"

/***************************************************************************************************
Local Function declaration
***************************************************************************************************/
static void i2c_Clock(void);
static void i2c_Ack(void);
static void i2c_NoAck(void);
/**************************************************************************************************/

/***************************************************************************************************
void I2C_Init(void)
****************************************************************************************************
I/P Arguments: none.
Return value  : none
description  :This function is used to initialize the I2C module.
***************************************************************************************************/
void eeI2C_Init(void) {}

/***************************************************************************************************
void I2C_Start(void)
****************************************************************************************************
I/P Arguments: none.
Return value  : none
description  :This function is used to generate I2C Start Condition.
Start Condition: SDA goes low when SCL is High.
____________
SCL:          |            |
________|            |______
_________
SDA:      |         |
____|         |____________
***************************************************************************************************/
void eeI2C_Start(void)
{
  SCL_PIN_LOW;  // Pull SCL low
  SDA_PIN_HIGH; // Pull SDA High
  DELAY_us(1);
  SCL_PIN_HIGH; //Pull SCL high
  DELAY_us(1);
  SDA_PIN_LOW; //Now Pull SDA LOW, to generate the Start Condition
  DELAY_us(1);
  SCL_PIN_LOW; //Finally Clear the SCL to complete the cycle
}

/***************************************************************************************************
void I2C_Stop(void)
****************************************************************************************************
I/P Arguments: none.
Return value  : none
description  :This function is used to generate I2C Stop Condition.
Stop Condition: SDA goes High when SCL is High.
____________
SCL:          |            |
________|            |______
_________________
SDA:            |
__________|
***************************************************************************************************/

void eeI2C_Stop(void)
{
  SCL_PIN_LOW; // Pull SCL low
  DELAY_us(1);
  SDA_PIN_LOW; // Pull SDA  low
  DELAY_us(1);
  SCL_PIN_HIGH; // Pull SCL High
  DELAY_us(1);
  SDA_PIN_HIGH; // Now Pull SDA High, to generate the Stop Condition
}

/***************************************************************************************************
void I2C_Write(uint8_t v_i2cData_u8)
****************************************************************************************************
I/P Arguments: uint8_t-->8bit data to be sent.
Return value  : none
description  :This function is used to send a byte on SDA line using I2C protocol
8bit data is sent bit-by-bit on each clock cycle.
MSB(bit) is sent first and LSB(bit) is sent at last.
Data is sent when SCL is low.
___     ___     ___     ___     ___     ___     ___     ___     ___     ___
SCL:   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
__|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___
SDA:    D8       D7     D6      D5      D4       D3      D2      D1      D0     ACK
***************************************************************************************************/
void eeI2C_Write(uint8_t v_i2cData_u8)
{
  uint8_t i;
  for (i = 0; i < 8; i++) // loop 8 times to send 1-byte of data
  {
    if (v_i2cData_u8 & 0x80)
      SDA_PIN_HIGH;
    else
      SDA_PIN_LOW;
    i2c_Clock();        // Generate Clock at SCL
    v_i2cData_u8 <<= 1; // Bring the next bit to be transmitted to MSB position
  }
  i2c_Clock();
}

/***************************************************************************************************
uint8_t I2C_Read(uint8_t v_ackOption_u8)
****************************************************************************************************
I/P Arguments: none.
Return value  : uint8_t(received byte)
description :This fun is used to receive a byte on SDA line using I2C protocol.
8bit data is received bit-by-bit each clock and finally packed into Byte.
MSB(bit) is received first and LSB(bit) is received at last.
___     ___     ___     ___     ___     ___     ___     ___     ___     ___
SCL:    |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
__|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |__
SDA:    D8       D7     D6      D5       D4     D3       D2      D1     D0      ACK
***************************************************************************************************/
uint8_t eeI2C_Read(uint8_t v_ackOption_u8)
{
  uint8_t v_i2cData_u8 = 0x00;

  SDA_PIN_HIGH;                   //Make SDA as I/P
  for (uint8_t i = 0; i < 8; i++) // loop 8times read 1-byte of data
  {
    DELAY_us(1);
    SCL_PIN_HIGH; // Pull SCL High
    DELAY_us(1);

    v_i2cData_u8 = v_i2cData_u8 << 1;      //v_i2cData_u8 is Shifted each time and
    v_i2cData_u8 = v_i2cData_u8 | SDA_PIN; //ORed with the received bit to pack into byte

    SCL_PIN_LOW; // Clear SCL to complete the Clock
  }

  if (v_ackOption_u8) /*Send the Ack/NoAck depending on the user option*/
    i2c_Ack();
  else
    i2c_NoAck();

  return v_i2cData_u8; // Finally return the received Byte*
}

/***************************************************************************************************
Local functions
***************************************************************************************************/

/***************************************************************************************************
static void i2c_Clock()
****************************************************************************************************
I/P Arguments: none.
Return value  : none
description  :This function is used to generate a clock pulse on SCL line.
***************************************************************************************************/
static void i2c_Clock(void)
{
  DELAY_us(1);
  SCL_PIN_HIGH; // Wait for Some time and Pull the SCL line High
  DELAY_us(1);  // Wait for Some time
  SCL_PIN_LOW;  // Pull back the SCL line low to Generate a clock pulse
}

/***************************************************************************************************
static void i2c_Ack()
****************************************************************************************************
I/P Arguments: none.
Return value  : none
description  :This function is used to generate a the Positive ACK
pulse on SDA after receiving a byte.
***************************************************************************************************/
static void i2c_Ack(void)
{
  SDA_PIN_LOW;  // Pull SDA low to indicate Positive ACK
  i2c_Clock();  // Generate the Clock
  SDA_PIN_HIGH; // Pull SDA back to High(IDLE state)
}

/***************************************************************************************************
static void i2c_NoAck()
****************************************************************************************************
I/P Arguments: none.
Return value  : none
description  :This function is used to generate a the Negative/NO ACK
pulse on SDA after receiving all bytes.
***************************************************************************************************/
static void i2c_NoAck(void)
{
  SDA_PIN_HIGH; // Pull SDA high to indicate Negative/NO ACK
  i2c_Clock();  // Generate the Clock
  SCL_PIN_HIGH; // Set SCL
}
