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
#ifndef COMMON_H
#define COMMON_H

/*! \file common.h
    \brief file with the common data and configuration.
    
*/

#include <Wire.h>

//! DEBUG Flag
#define DEBUG_MODE		false


/** \brief Types of commands	*/
#define QUERY	0
#define STREAM	1

/** \brief Different types of messages */
#define REQUEST_RANGES		0
#define REQUEST_NOSE 		1
#define REQUEST_TPA81		2
#define REQUEST_ANEMOMETER	3
#define REQUEST_NUNCHUCK	4
#define REQUEST_SHTXX		5


#define NUMBER_OF_DIFF_SENSORS  4

struct sInfo
{
  unsigned char sensorType;
  char sensorAddress[5];
  unsigned char numSensors;
};

struct sInfo sensorInfo[NUMBER_OF_DIFF_SENSORS];

//extern unsigned char NCHUCKbuffer[6];

#endif // COMMON_H
