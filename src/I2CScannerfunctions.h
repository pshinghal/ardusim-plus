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
#ifndef I2CSCANNERFUNCTIONS_H
#define I2CSCANNERFUNCTIONS_H

/**
 * I2CScannerfunction.h -- I2C bus scanner for Arduino
 *
 * based on: Tod E. Kurt, http://todbot.com/blog/, 2009
 *
 */

#include "Arduino.h"
#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#include "common.h"


/* SRF08
 0xE0 0xE2 0xE4 0xE6 0xE8 0xEA 0xEC 0xEE 0xF0 0xF2 0xF4 0xF6 0xF8 0xFA 0xFC 0xFE
 0x70 0x71 0x72 0x73 0x74 0x75 0x76 0x77 0x78 0x79 0x7A 0x7B 0x7C 0x7D 0x7E 0x7F
 */
#define SRF08_START_ADDR     0x70
#define SRF08_END_ADDR       0x7F

/* NOSE
 0x49 0x4A 0x4B
 */
#define NOSE_START_ADDR     0x49
#define NOSE_END_ADDR       0x4B

/* TPA81
 0xD0, 0xD2, 0xD4, 0xD6, 0xD8, 0xDA, 0xDC, 0xDE
 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F
 */
#define TPA81_START_ADDR     0x68
#define TPA81_END_ADDR       0x6F

/* ANEMOMETER
 0x48
 */
#define ANEMOMETER_START_ADDR     0x48
#define ANEMOMETER_END_ADDR       0x48



void addSensorInformation(struct sInfo *ptrSensorInfo, byte *address, int len)
{
    int i, index = 0;
    int nRANGES = 0;
    int nNOSE = 0;
    int nTPA81 = 0;

    for( i = 0; i < len; i++ )
    {
        // check the type
        if( *(address+i) >= SRF08_START_ADDR && *(address+i) <= SRF08_END_ADDR)
        {
            //Serial.println("It is a SONAR");
            // SONAR
            ptrSensorInfo[0].sensorType = REQUEST_RANGES;
            ptrSensorInfo[0].sensorAddress[nRANGES] = *(address+i);
            nRANGES++;
            ptrSensorInfo[0].numSensors = nRANGES;
        }
        else if( *(address+i) >= NOSE_START_ADDR && *(address+i) <= NOSE_END_ADDR)
        {
            //Serial.println("It is a NOSE");
            // NOSE
            ptrSensorInfo[1].sensorType = REQUEST_NOSE;
            ptrSensorInfo[1].sensorAddress[nNOSE] = *(address+i);
            nNOSE++;
            ptrSensorInfo[1].numSensors = nNOSE;
        }
        else if( *(address+i) >= TPA81_START_ADDR && *(address+i) <= TPA81_END_ADDR)
        {
            //Serial.println("It is a TPA81");
            // TPA81
            ptrSensorInfo[2].sensorType = REQUEST_TPA81;
            ptrSensorInfo[2].sensorAddress[nTPA81] = *(address+i);
            nTPA81++;
            ptrSensorInfo[2].numSensors = nTPA81;
        }
        else if( *(address+i) >= ANEMOMETER_START_ADDR && *(address+i) <= ANEMOMETER_END_ADDR)
        {
            //Serial.println("It is a ANEMOMETER");
            // ANEMOMETER
            ptrSensorInfo[3].sensorType = REQUEST_ANEMOMETER;
            ptrSensorInfo[3].sensorAddress[0] = *(address+i);
            ptrSensorInfo[3].numSensors = 1;
        }
    }  // end for
}


void printSensorInformation(struct sInfo *ptrSensorInfo, int size)
{
    int i, j, index = 0;

    Serial.println("-- Sensor Information --");

    for( i = 0; i < size; i++ )
    {
        if( ptrSensorInfo[i].numSensors != 0 )
        {
            Serial.print("Type: ");
            Serial.println(ptrSensorInfo[i].sensorType,DEC);
            Serial.print("Nsensors: ");
            Serial.println(ptrSensorInfo[i].numSensors,DEC);
            Serial.println("Address: ");

            for( j = 0; j < ptrSensorInfo[i].numSensors; j++ )
                Serial.println(ptrSensorInfo[i].sensorAddress[j],HEX);

        }
        else
            Serial.println("No sensor detected of this type.");

    }
}



/** Scan the I2C bus between addresses from_addr and to_addr.
 * On each address, call the callback function with the address and result.
 * If result==0, address was found, otherwise, address wasn't found
 * (can use result to potentially get other status on the I2C bus, see twi.c)
 * Assumes Wire.begin() has already been called
 */
void scanI2CBus(byte from_addr, byte to_addr, void(*callback)(byte address, byte result) ) 
{
    if (DEBUG_MODE)
    {
        Serial.print("starting scanning of I2C bus from ");
        Serial.print(from_addr,HEX);
        Serial.print(" to ");
        Serial.print(to_addr,HEX);
        Serial.println("...");
    }

    byte addrFound[10];
    byte idx = 0;
    byte rc;
    byte data = 0; // not used, just an address to feed to twi_writeTo()
    for( byte addr = from_addr; addr <= to_addr; addr++ )
    {
        rc = twi_writeTo(addr, &data, 0, 1);

        if(rc == 0)
        {
            addrFound[idx] = addr;
            idx++;
        }

        if (DEBUG_MODE)
            callback( addr, rc );
    }

    if (DEBUG_MODE)
        Serial.println("\ndone");

    // Add to the information struct
    addSensorInformation( sensorInfo, addrFound, idx);

    // print
    if (DEBUG_MODE)
        printSensorInformation(sensorInfo, 4);

}

/** Called when address is found in scanI2CBus()
 * Feel free to change this as needed
 * (like adding I2C comm code to figure out what kind of I2C device is there)
 */
void scanFunc( byte addr, byte result )
{
    Serial.print("addr: ");
    Serial.print(addr,HEX);
    Serial.print( (result==0) ? " found!":"       ");
    Serial.println( (addr%4) ? "\t":"\n");
}

#endif // I2CSCANNERFUNCTIONS_H


