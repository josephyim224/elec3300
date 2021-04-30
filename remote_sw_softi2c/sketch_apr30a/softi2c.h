/***************************************************************************************************
                                    ExploreEmbedded Copyright Notice
****************************************************************************************************
   File:   stdutils.h
   Version: 15.0
   Author: ExploreEmbedded
   Website: http://www.exploreembedded.com/wiki
   Description: Contains function prototypes for I2c routines.
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
 * Modified from generic 8051 to Arduino atmega32u4.
 */

#ifndef _I2C_H
#define _I2C_H

// #include <reg51.h>
// #include "stdutils.h"

typedef unsigned char uint8_t;

/***************************************************************************************************
                               SCL and SDA pin configuration
***************************************************************************************************/
// sbit SCL_PIN = P0 ^ 6; // SCL Connected to P0.6
// sbit SDA_PIN = P0 ^ 7; // SDA Connected to P0.7

// #define SCL_Direction SCL_PIN
// #define SDA_Direction SDA_PIN

#define SCL_PIN_HIGH digitalWrite(8, 1)
#define SCL_PIN_LOW digitalWrite(8, 0)

#define SDA_PIN_HIGH digitalWrite(9, 1)
#define SDA_PIN_LOW digitalWrite(9, 0)
#define SDA_PIN_WRITE(v) digitalWrite(9, v)

#define SDA_PIN digitalRead(8)
#define SCL_PIN digitalRead(9)

#define DELAY_us(n) delayMicroseconds(1);

/**************************************************************************************************/

/***************************************************************************************************
                             Function Prototypes
***************************************************************************************************/
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t);
uint8_t I2C_Read(uint8_t);
/**************************************************************************************************/

#endif
