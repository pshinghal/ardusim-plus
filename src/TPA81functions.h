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
#ifndef TPA81FUNCTIONS_H
#define TPA81FUNCTIONS_H

/*! \file TPA81functions.h
 \brief Functions to be used with TPA81 sensor by I2C.
 
 This file describes a set of function to perform the initialization,
 configuration, and read and write operations.
 */

#include "Arduino.h"
#include <Wire.h>

int getTPA81(char addr, char TPA81reg)
{
    if (DEBUG_MODE)
        Serial.println("I2C > Getting TPA81 Temperature...");

    time[0] = millis();
    waitFlag = 1;

    // Function to get light reading
    Wire.beginTransmission(addr);
    Wire.write(byte(TPA81reg));                           // Call register to get light reading
    Wire.endTransmission();

    Wire.requestFrom(addr, 1);                     // Request 1 byte

    while(waitFlag)
    {
        time[1] = millis(); 

        if(time[1] - time[0] > 1)
            waitFlag = FALSE;
    }

    int tempRead = Wire.read();                 // Get light reading
    return(tempRead);                              // Returns lightRead
}

#endif // TPA81FUNCTIONS_H

