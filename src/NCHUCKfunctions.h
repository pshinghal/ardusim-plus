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
#ifndef NCHUCKFUNCTIONS_H
#define NCHUCKFUNCTIONS_H

#include "Arduino.h"

// Nunchuck functions

unsigned char NCHUCKbuffer[6];

/** \brief Nunchuck Data Buffer 				*/
int NCHUCKdata[7] = {
    0, 0, 0, 0, 0, 0, 0};

//! Functions prototypes
static void NCHUCKsetpowerpins();
unsigned char NCHUCKdecode_byte (unsigned char byte);
void NCHUCKinit();
void NCHUCKsend_request();
int NCHUCKget_data();
void parseNCdata(unsigned char *ibuffer, int *obuffer);
void getNCHUCK();


// Uses port C (analog in) pins as power & ground for Nunchuck
static void NCHUCKsetpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
    DDRC |= _BV(pwrpin) | _BV(gndpin);
    PORTC &=~ _BV(gndpin);
    PORTC |=  _BV(pwrpin);
    delay(100);  // wait for things to stabilize        
}


unsigned char NCHUCKdecode_byte (unsigned char byte)
{
    return (byte ^ 0x17) + 0x17;
    ;
}


void NCHUCKinit()
{
    Wire.begin();
    Wire.beginTransmission(0x52);
    Wire.write(byte(0x40));
    Wire.write(byte(0x00));
    Wire.endTransmission();
}


void NCHUCKsend_request()
{
    Wire.beginTransmission(0x52);
    Wire.write(byte(0x00));
    Wire.endTransmission();
}


int NCHUCKget_data()
{
    int cnt=0;

    NCHUCKsend_request();  // send request for next data payload

    delayMicroseconds(1000);

    Wire.requestFrom (0x52, 6);	// request data from nunchuck
    while( Wire.available () ) 
    {
        // receive byte as an integer
        NCHUCKbuffer[cnt] = NCHUCKdecode_byte(Wire.read());
        cnt++;
    }

    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5)
        return 1;   // success
    return 0; 		//failure
}


// Parse Nunchuck Data
void parseNCdata(unsigned char *ibuffer, int *obuffer)
{
    obuffer[0] = ibuffer[0];
    obuffer[1] = ibuffer[1];

    if ((ibuffer[5] & 0x01)!=0)
        obuffer[2] = 1;
    else
        obuffer[2] = 0;
    if ((ibuffer[5] & 0x02)!=0)
        obuffer[3] = 1;
    else
        obuffer[3] = 0;

    obuffer[4] = (ibuffer[2]) << 2;
    obuffer[5] = (ibuffer[3]) << 2;
    obuffer[6] = (ibuffer[4]) << 2;

    obuffer[4] += ((ibuffer[5]) >> 2) & 0x03;
    obuffer[5] += ((ibuffer[5]) >> 4) & 0x03;
    obuffer[6] += ((ibuffer[5]) >> 6) & 0x03;	
}


void getNCHUCK()
{
    char status = 0;

    if (DEBUG_MODE)
        Serial.println("I2C > Getting NUNCHUCK Data...");

    status = NCHUCKget_data();

    if (status)
        parseNCdata(NCHUCKbuffer, NCHUCKdata);
    else
    {
        NCHUCKbuffer[0] = 1;
        NCHUCKbuffer[1] = 2;
        NCHUCKbuffer[2] = 3;
        NCHUCKbuffer[3] = 4;
        NCHUCKbuffer[4] = 5;
        NCHUCKbuffer[5] = 6;
        NCHUCKbuffer[6] = 7;
    }
}

#endif // NCHUCKFUNCTIONS_H

