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
* Author: Ricardo Faria rfaria@isr.uc.pt
*********************************************************************/
#ifndef SRF08FUNCTIONS_H
#define SRF08FUNCTIONS_H

/*! \file SRF08functions.h
 \brief Functions to be used with SRF08 SONAR by I2C.
 
 This file describes a set of function to perform the initialization,
 configuration, and read and write operations.
 */

#include "Arduino.h"
#include <Wire.h>



/** \brief I2C SRF08 Registers 					*/
#define cmdReg 		0x00          // Command Register
#define cmdByte 	0x51          // Command byte
#define lightReg 	0x01          // Byte to read light sensor
#define rangeReg 	0x02          // Byte for start of ranging data


/** \brief I2C MSB and LSB Bytes  				*/
byte highByte = 0x00;       // Stores high byte from ranging
byte lowByte = 0x00;        // Stored low byte from ranging



void startRange(unsigned char addr)
{
    Wire.beginTransmission(addr);        // Start communticating with SRF08
    Wire.write(byte(cmdReg));                   // Send Command Byte
    Wire.write(byte(0x51));                     // Send 0x51 to start a ranging
    Wire.endTransmission();
}

void setParameter(unsigned char addr, int reg, int val)
{
    Wire.beginTransmission(addr);        // Start communticating with SRF08
    Wire.write(byte(reg));                     // Send Command Byte
    Wire.write(byte(val));                     // Send 0x51 to start a ranging
    Wire.endTransmission();
}


//setSensitivity() sets the max range
void setSensitivity(int address)
{
    //start i2c transmission
    Wire.beginTransmission(address);

    //send command
    Wire.write(byte(0x01)); //set to reg1
    Wire.write(byte(0x00)); //hex code for sensitity (0×00 = 94)

    //end i2c trans
    Wire.endTransmission();
}


//setRange() sets the max range
void setRange(int address)
{
    //start i2c transmission
    Wire.beginTransmission(address);

    //send command
    Wire.write(byte(0x02)); //set to reg2
    Wire.write(byte(0xFF)); //hex code for range (0xFF = max avail)

    //end i2c trans
    Wire.endTransmission();
}


void setAddr(unsigned char addr, unsigned char newaddr)
{
    if (DEBUG_MODE)
        Serial.println("I2C > Setting SRF Address...");

    Wire.beginTransmission(addr);        // Start communticating with SRF08
    Wire.write(byte(cmdReg));                   // Send Command Byte
    Wire.write(byte(0xA0));                     // Send 0xA0
    Wire.endTransmission();
    Wire.beginTransmission(addr);        // Start communticating with SRF08
    Wire.write(byte(cmdReg));                   // Send Command Byte
    Wire.write(byte(0xAA));                     // Send 0xAA
    Wire.endTransmission();
    Wire.beginTransmission(addr);        // Start communticating with SRF08
    Wire.write(byte(cmdReg));                   // Send Command Byte
    Wire.write(byte(0xA5));                     // Send 0xA5
    Wire.endTransmission();
    Wire.beginTransmission(addr);        // Start communticating with SRF08
    Wire.write(byte(cmdReg));                   // Send Command Byte
    Wire.write(byte(newaddr));                     // Send 0xA0
    Wire.endTransmission();
}


int getRange(char addr)
{
    if (DEBUG_MODE)
        Serial.println("I2C > Getting SRF Range...");

    time[0] = millis();
    waitFlag = 1;

    // This function gets a ranging from the SRF08  
    int range = 0; 

    Wire.beginTransmission(addr);                  // Start communicating with SRFmodule
    Wire.write(byte(rangeReg));                           // Call the register for start of ranging data
    Wire.endTransmission();

    Wire.requestFrom(addr, 2);                      // Request 2 bytes from SRF module

    while(waitFlag)
    {
        time[1] = millis(); 

        if(time[1] - time[0] > 1)
            waitFlag = FALSE;
    }

    highByte = Wire.read();                      // Get high byte
    lowByte = Wire.read();                       // Get low byte

        range = (highByte << 8) + lowByte;              // Put them together

    return(range);                                  // Returns Range
}


int getLight(char addr)
{
    if (DEBUG_MODE)
        Serial.println("I2C > Getting SRF Light...");

    time[0] = millis();
    waitFlag = 1;

    // Function to get light reading
    Wire.beginTransmission(addr);
    Wire.write(byte(lightReg));                           // Call register to get light reading
    Wire.endTransmission();

    Wire.requestFrom(addr, 1);                     // Request 1 byte

    while(waitFlag)
    {
        time[1] = millis(); 

        if(time[1] - time[0] > 1)
            waitFlag = FALSE;
    }

    int lightRead = Wire.read();                 // Get light reading
    return(lightRead);                              // Returns lightRead
}


int getSoft(char addr)
{
    if (DEBUG_MODE)
        Serial.println("I2C > Getting SRF Software Version...");

    time[0] = millis();
    waitFlag = 1;  

    // Function to get software revision
    Wire.beginTransmission(addr);                  // Begin communication with the SRF module
    Wire.write(byte(cmdReg));                             // Sends the command bit, when this bit is read it returns the software revision
    Wire.endTransmission();

    Wire.requestFrom(addr, 1);                      // Request 1 byte

    while(waitFlag)
    {
        time[1] = millis(); 
        if(time[1] - time[0] > 1)
            waitFlag = FALSE;
    }

    int software = Wire.read();                  // Get byte
    return(software);
}

#endif // SRF08FUNCTIONS_H

