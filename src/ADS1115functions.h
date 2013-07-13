/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Pedro Sousa pvsousa@isr.uc.pt
*********************************************************************/
#ifndef ADS1115FUNCTIONS_H
#define ADS1115FUNCTIONS_H

/*! \file ADS1115functions.h
 \brief Functions to be used with ADS1115 ADC by I2C.
 
 This file describes a set of function to perform the initialization,
 configuration, and read and write operations.
 */

#include "Arduino.h"
#include <Wire.h>

#include "common.h"

// ADS1115 functions

//#define DEBUG_MODE		false

#define READ  	1
#define WRITE 	0

#define TRUE 	1
#define FALSE 	0

/** \brief Define the current sensor - Chemical		*/
#define NOSE    0
/** \brief Define the current sensor - Anemometer	*/
#define ANEM    1

/** \brief ADS1115 I2C Address (7 bits)			*/
#define NOSE_ADDR    0x49

/** \brief Anemometer I2C Address				*/
#define ANEM_ADDR	0x48

/** \brief I2C ADS1115 Registers				*/
#define convREG        0x00 	// conversion Register
#define configREG      0x01 	// configuration Register
#define lowThreshREG   0x02		// low threshold value Register
#define highThreshREG  0x03		// high threshold value Register

/** \brief ADS1115 Analog Channels for Gas Sensors		*/
#define CH0  0
#define CH1  1
#define CH2  2
#define CH3  3

/** \brief ADS1115 Analog Channels for Anemometer		*/
#define CH0_anem  4
#define CH1_anem  5
#define CH2_anem  6
#define CH3_anem  7



/** \brief  Config Register Organization
 *	OS | MUX2 | MUX1 | MUX0 | PGA2 | PGA1 | PGA0 | MODE
 *	DR2 | DR1 | DR0 | COMP_MODE | COMP_POL | COMP_LAT | COMP_QUE1 | COMP_QUE0
 *
 *	register for single value, AN0 to GND FS = 4.096V, DR to 128 SPS
 *               C  3  8  3
 */
#define An0ConfigREG_H 	0b11000011
#define An0ConfigREG_L 	0b10000011

/** \brief Register for single value, AN1 to GND FS = 4.096V, DR to 128 SPS
 *               D  3  8  3
 */
#define An1ConfigREG_H 	0b11010011
#define An1ConfigREG_L 	0b10000011

/** \brief Register for single value, AN2 to GND FS = 4.096V, DR to 128 SPS
 *               E  3  8  3
 */
#define An2ConfigREG_H 	0b11100011
#define An2ConfigREG_L 	0b10000011

/** \brief Register for single value, AN3 to GND FS = 4.096V, DR to 128 SPS
 *               F  3  8  3
 */
#define An3ConfigREG_H 	0b11110011
#define An3ConfigREG_L 	0b10000011


/** \brief Register for single value, AN0 to GND FS = 6.144V, DR to 128 SPS
 *               C  1  8  3
 */
#define AnemAn0ConfigREG_H 	0b11000001
#define AnemAn0ConfigREG_L 	0b10000011

/** \brief Register for single value, AN1 to GND FS = 6.144V, DR to 128 SPS
 *               D  1  8  3
 */
#define AnemAn1ConfigREG_H 	0b11010001
#define AnemAn1ConfigREG_L 	0b10000011

/** \brief Register for single value, AN2 to GND FS = 6.144V, DR to 128 SPS
 *               E  1  8  3
 */
#define AnemAn2ConfigREG_H 	0b11100001
#define AnemAn2ConfigREG_L 	0b10000011

/** \brief Register for single value, AN3 to GND FS = 6.144V, DR to 128 SPS
 *               F  1  8  3
 */
#define AnemAn3ConfigREG_H 	0b11110001
#define AnemAn3ConfigREG_L 	0b10000011


extern char waitFlag;

/** \brief Time Buffer 							*/
extern unsigned long time[4];


/*! \fn void ADS1115_startConv(char addr, char channel)
 \brief Starts a new convertion fron CH channel and address addr.
 
 \param addr The I2C device address.
 \param channel The required sensor channel.
 */
void ADS1115_startConv(char addr, char channel);

/*! \fn ADS1115_getData(char addr)
 \brief Starts a new convertion fron CH channel and address addr.
 
 \param addr The I2C device address.
 \return The previously acquired/converted data.
 */
int ADS1115_getData(char addr);

/*! \fn voltageConverter(int data)
 \brief Convertes the sensor values to voltage (in mV).
 
 \param data The data to be converted.
 \return The converted data.
 */
int voltageConverter(int data); 

/*! \fn readConfigRegister(char addr)
 \brief Reads the current data in the configuration Word (2 bytes register).
 
 \param addr The I2C device address.
 \return The current configuration data.
 */
int readConfigRegister(char addr);



void ADS1115_startConv(char addr, char channel)
{
    char HighReg;
    char LowReg;

    Wire.beginTransmission(addr);    // Start communticating with ADS1115
    Wire.write(configREG);                    // Send Config Byte

    // choose channel to read from
    switch ( channel )
    {
    case CH0:
        HighReg = An0ConfigREG_H;
        LowReg = An0ConfigREG_L;
        break;
    case CH1:
        HighReg = An1ConfigREG_H;
        LowReg = An1ConfigREG_L;
        break;
    case CH2:
        HighReg = An2ConfigREG_H;
        LowReg = An2ConfigREG_L;
        break;
    case CH0_anem:
        HighReg = AnemAn0ConfigREG_H;
        LowReg = AnemAn0ConfigREG_L;
        break;
    case CH1_anem:
        HighReg = AnemAn1ConfigREG_H;
        LowReg = AnemAn1ConfigREG_L;
        break;
    case CH2_anem:
        HighReg = AnemAn2ConfigREG_H;
        LowReg = AnemAn2ConfigREG_L;
        break;
    case CH3_anem:
        HighReg = AnemAn3ConfigREG_H;
        LowReg = AnemAn3ConfigREG_L;
        break;
    default:
        HighReg = An0ConfigREG_H;
        LowReg = An0ConfigREG_L;
        break;
    }

    Wire.write(HighReg);                      // Send Config data High Byte
    Wire.write(LowReg);                       // Send Config data Low Byte
    Wire.endTransmission();
}

int ADS1115_getData(char addr)
{
    char HighByte;
    char LowByte;

    if (DEBUG_MODE)
        Serial.println("I2C > Getting ADS1115 Data...");

    time[0] = millis();
    waitFlag = 1;

    // get data from ADS1115  
    int ADC_data = 0; 

    Wire.beginTransmission(addr);      // Start communicating with ADS1115
    Wire.write(byte(convREG));                        // Call the register for start of ranging data
    Wire.endTransmission();

    Wire.requestFrom(addr, 2);          // Request 2 bytes from SRF module

    delayMicroseconds(10);

    HighByte = Wire.read();                  // Get high byte
    LowByte = Wire.read();                   // Get low byte

        ADC_data = (HighByte << 8) + LowByte;       // Put them together
    return(ADC_data);                           // Returns Range
}

int voltageConverter(int data)
{
    int conv_data;
    conv_data = data / 8;

    return conv_data;
} 

int readConfigRegister(char addr)
{
    byte HighByte;
    byte LowByte;
    int configWord;

    Wire.beginTransmission(addr);    // Start communicating with ADS1115
    Wire.write(byte(configREG));                    // Call the register for start of ranging data
    Wire.endTransmission();

    Wire.requestFrom(addr, 2);

    delayMicroseconds(10);

    HighByte = Wire.read();                // Get high byte
    LowByte = Wire.read();                 // Get low byte

        configWord = (HighByte << 8) + LowByte;   // Put them together

    return configWord;
}

#endif // ADS1115FUNCTIONS_H

