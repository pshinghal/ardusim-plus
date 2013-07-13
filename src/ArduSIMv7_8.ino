
/*! \file RoombaSIMv7.pde
 \brief Main program for arduino board.
 
 */

#include <Wire.h>
//#include <SHT1x.h>


#include "ADS1115functions.h"

#include "SRF08functions.h"

#include "TPA81functions.h"

#include "NCHUCKfunctions.h"

#include "common.h"

#include "I2CScannerfunctions.h"

#define LEDpin 	13

#define READ  	1
#define WRITE 	0

#define TRUE 	1
#define FALSE 	0

// ARDUSIM version - format 10 = 1.0
#define ARDUSIM_VERSION 10

//! DEBUG Flag
//#define DEBUG_MODE		false


/** \brief Comunication Protocol Variables	*/
#define DATA_CHAR 		0x40	// @
#define TERM_CHAR 		0x65	// e

/** \brief Different types of messages */
#define REQUEST_RANGES		0
#define REQUEST_NOSE 		1
#define REQUEST_TPA81		2
#define REQUEST_ANEMOMETER	3
#define REQUEST_NUNCHUCK	4
#define REQUEST_SHTXX		5
#define REQUEST_VERSION		6

#define QUERY_MODE     'Q'
#define STREAM_MODE    'S'
#define INFO_MODE      'I'
#define AUTO_MODE      'A'
#define VERSION_MODE   'V'

/** \brief Types of commands	*/
#define QUERY	0
#define STREAM	1
#define INFO    2
#define AUTO    3
#define VERSION 4


/** \brief Specify the amount of connected sensors	*/
#define NUMBER_OF_RANGES      5
#define NUMBER_OF_NOSE        2
#define NUMBER_OF_TPA81       1 
#define NUMBER_OF_ANEMOMETER  1
#define NUMBER_OF_SHTXX       1


/** \brief SRF08 I2C Address Array (7 bits)  */
char SRF08addr[5] = {
    0x70, 0x73, 0x72, 0x74, 0x71};

/** \brief Chemical sensor ADS1115 I2C Address (7 bits)	*/
#define ADS1115_ADDR    0x49

/** \brief Anemometer ADS1115 I2C Address (7 bits)	*/
#define ANEM_ADS1115_ADDR    0x48

/** \brief TPA81 I2C Address (7 bits)	*/
#define TPA81_ADDR	0x68

/** \brief Anemometer I2C Address        */
#define ANEM_ADDR	0xAA 


//! Data Buffers
/** \brief TPA81 Temperatue Buffer        */
unsigned char TPA81data[9] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0};

/** \brief Anemometer Velocity Buffer			*/
int ANEMdata[4] = {
    0, 0, 0, 0};

/** \brief SRF08 Range Buffer 				*/
unsigned int SRF08data[5] = {
    0, 0, 0, 0, 0};

/** \brief Time Buffer 					*/
unsigned long time[4];

/** \brief ADS1115 buffers 				*/
int ADS1115data[3];
int conv_ADC_data;
int ADC_data;


/** \brief UART Control Variables 			*/
char u1_data_ready;
char u1_com_mode;
char i1;

/** \brief UART RX Buffer 				*/
char u1_rxdata[20];

/** \brief UART Buffer and Control Flags		*/
char serial[128];
char waitFlag;


/** \brief Time Buffer 					*/
unsigned long lastTime, currentTime;



/** \brief command buffer    */
char cmd_str[64];
int mode;


/** \brief check the presence of sensors connected to arduino    */
bool activeRANGES = false;
bool activeNOSE = true;
bool activeTPA81 = false;
bool activeANEMOMETER = false;
bool activeSHTXX = false;

/** \brief number of sensors connected to arduino    */
unsigned char numberOfRANGES = NUMBER_OF_RANGES;
unsigned char numberOfNOSE = NUMBER_OF_NOSE;
unsigned char numberOfTPA81 = NUMBER_OF_TPA81;
unsigned char numberOfANEMOMETER = NUMBER_OF_ANEMOMETER;
unsigned char numberOfSHTXX = NUMBER_OF_SHTXX;
// version
//unsigned char ardusimVersion = ARDUSIM_VERSION;

// Specify data and clock connections and instantiate SHT1x object
// MEGA DIgital ports
#define dataPin  22
#define clockPin 23

//SHT1x sht1x(dataPin, clockPin);


void setup()
{
    // I2C
    Wire.begin();

    // UART
    Serial.begin(57600);

    // Nunchuck
    // NCHUCKsetpowerpins();
    NCHUCKinit();

    // LED
    pinMode(LEDpin, OUTPUT);

    if (DEBUG_MODE)
        Serial.println("Setup OK...");

    // set gain
    setParameter(0xE0, 0x01, 0x00);
    //setSensitivity(0xE0);
    delay(100);
    // set range
    setParameter(0xE0, 0x02, 0x2D);
    //setRange(0xE0);

    byte start_address = 1;
    byte end_address = 0x7F;

    if (DEBUG_MODE)
        Serial.println("\nI2CScanner ready!");

    // start the scan, will call "scanFunc()" on result from each address
    scanI2CBus( start_address, end_address, scanFunc );

    // Get current time
    lastTime = millis();

    delay(100);
}


//void parse( char *record, char *delim, char arr[][MAXFLDSIZE],int *fldcnt)
void parseRecData( char *record, char *delim, char arr[][16], int *msgArray, int *fldcnt)
{
    char*p = strtok(record, delim);
    int fld = 0;

    while(p)
    {
        strcpy(arr[fld], p);
        *(msgArray+fld) = atoi(arr[fld]);
        fld++;
        p = strtok('\0',delim);
    }		
    *fldcnt = fld;
    	
    if(DEBUG_MODE)
    {
        for(int i = 0; i < fld; i++)
     	{
     	    sprintf(serial,"Field number: %3d==%s   int:%d", i, arr[i], *(msgArray+i));
     	    Serial.println(serial);
     	}
     }
}


void clearBuffer(void * in)
{
    memset(in, 0, sizeof(in));
}


void loop()
{
    digitalWrite(LEDpin, LOW);

    // Check for available DATA
    if ( Serial.available() ) 
    {
        int inByte = Serial.read();

        // Check if DATA is valid
        if (inByte == DATA_CHAR && u1_com_mode == 0)
        {
            i1 = 0;
            u1_com_mode = 1;
        }

        // Save DATA to buffer
        if (u1_com_mode == 1)
        {
            u1_rxdata[i1] = inByte;
            if (inByte == TERM_CHAR)
            {
                u1_rxdata[i1+1] = '\0';
                int index = i1;
                i1 = 0;
                u1_com_mode = 0;

                if (DEBUG_MODE)
                    Serial.println(u1_rxdata);

                //if (u1_rxdata[0] == DATA_CHAR && u1_rxdata[4] == TERM_CHAR)
                if (u1_rxdata[0] == DATA_CHAR && u1_rxdata[index] == TERM_CHAR)
                {
                    u1_data_ready = 1;
                    if (DEBUG_MODE)
                        Serial.println("VALID DATA > Parsing...");
                }
                else
                {
                    if (DEBUG_MODE)
                        Serial.println("INVALID DATA > Cleaning Buffer...");
                    memset(u1_rxdata,0,sizeof(u1_rxdata));
                }
            }
            else
                i1++;
        }
    }

    // Check if Request Received
    if (u1_data_ready)
    {
        if (DEBUG_MODE)
            Serial.println("UART DATA READY");

        memset(cmd_str,0,sizeof(cmd_str));

        // Remove INIT and TERM CHAR

        for(int j=0;j < strlen(u1_rxdata)-4 ; j++)
        {
            //Serial.println(j, DEC);
            cmd_str[j] = u1_rxdata[j+2];
        }

        char cmdType;
        int numberOfCmds;
        int updateRate;

        int numberFields;
        char arr[16][16];
        char aux[64];
        int msgType[16];

        bool newMessage = false;

        int msgLen = 0;
        int index  = 0;

        char ans[32];
        char completeMessage[128];
        char streamCompleteMessage[128];
        const char *init = "@,";
        const char *end = "e";

        memset(aux,0,sizeof(aux));
        memcpy(aux, cmd_str, sizeof(cmd_str));
        // Parse all row
        parseRecData(aux, ",", arr, msgType, &numberFields);
        /*    
         if(DEBUG_MODE)
         {
             for(int o = 0; o < numberFields; o++)
             {
                 sprintf(serial,"Fnumber: %d==%s   %d", o, arr[o], msgType[o]);
         	Serial.println(serial);
             }
        }
         */
        // Check the command for action
        cmdType = arr[0][0];

        //	sprintf(serial, "u1_rxdata:%s     cmd_str:%s     aux:%s", u1_rxdata, cmd_str, aux);
        //	Serial.println(serial);

        switch (cmdType)
        {
        case 'Q':
            mode = QUERY;
            numberOfCmds = atoi(arr[1]);
            index = 2;
            break;
        case 'S':
            mode = STREAM;
            numberOfCmds = atoi(arr[1]);
            updateRate = atoi(arr[2]);
            index = 3;
            newMessage = true;
            break;
        case INFO_MODE:
            mode = INFO;
            break;
        case AUTO_MODE:
            mode = AUTO;
            break;
        case VERSION_MODE:
            mode = VERSION;
            break;
        default:
            // UNKNOWN CMD
            sprintf(serial, "Command %c not recognized!", cmdType);
            Serial.println(serial);
            break;
        }

        // Beginning of the message
        clearBuffer(completeMessage);
        strcat(completeMessage, init);

        if(mode == QUERY || newMessage == true)
        {
            if( msgType[index] == REQUEST_RANGES )
            {
                if (DEBUG_MODE)
                    Serial.println("REQUEST_RANGES...");

                // SRF08 Range Scan
                for( int i = 0; i < 5; i++)
                {
                    startRange( SRF08addr[i] );
                    delay(70);
                    SRF08data[i] = getRange( SRF08addr[i] );
                }

                sprintf(ans, "%d,%d,%d,%d,%d,%d,%d,", msgType[index], numberOfRANGES, SRF08data[0], SRF08data[1], SRF08data[2], SRF08data[3], SRF08data[4]);

                strcat(completeMessage, ans);

                if (DEBUG_MODE)
                    Serial.println("DONE...");

                // Increment index
                index++;
                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }

            if( msgType[index] == REQUEST_NOSE )
            {
                if (DEBUG_MODE)
                    Serial.println("REQUEST_NOSE...");

                // ADS1115 Chemical Sampling
                for( int i = 0; i < 2; i++)
                {
                    // start conversion
                    ADS1115_startConv(ADS1115_ADDR, i);					// config and start the convertion
                    // get data
                    ADC_data = ADS1115_getData(ADS1115_ADDR);           // read sampled data
                    // convert to mV
                    ADS1115data[i] = voltageConverter(ADC_data);    	// store converted data
                    delay(10);
                }

                sprintf(ans, "%d,%d,%d,%d,", msgType[index], numberOfNOSE, ADS1115data[0], ADS1115data[1]);

                strcat(completeMessage, ans);

                if (DEBUG_MODE)
                    Serial.println("DONE...");

                // Increment index
                index++;
                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }

            if( msgType[index] == REQUEST_TPA81 )
            {
                if (DEBUG_MODE)
                    Serial.println("REQUEST_TPA81...");

                // TPA81 Temperature Sampling
                for(int i = 0; i < 9; i++)
                    TPA81data[i] = getTPA81(TPA81_ADDR, (i+1));

                sprintf(ans, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", msgType[index], numberOfTPA81, TPA81data[0], TPA81data[1], TPA81data[2], TPA81data[3], TPA81data[4], TPA81data[5], TPA81data[6], TPA81data[7], TPA81data[8]);

                strcat(completeMessage, ans);

                if (DEBUG_MODE)
                    Serial.println("DONE...");

                // Increment index
                index++;
                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( msgType[index] == REQUEST_ANEMOMETER )
            {
                if (DEBUG_MODE)
                    Serial.println("REQUEST_TPA81...");

                // Anemometer Sampling
                for (int i = 0; i < 4; i++ )
                    ANEMdata[i] = getANEM(ANEM_ADDR, 0x00);

                sprintf(ans, "%d,%d,%d,%d,%d,%d,", msgType[index], numberOfANEMOMETER, ANEMdata[0], ANEMdata[1], ANEMdata[2], ANEMdata[3] );

                strcat(completeMessage, ans);

                if (DEBUG_MODE)
                    Serial.println("DONE...");

                // Increment index
                index++;
                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }

            if (msgType[index] == REQUEST_NUNCHUCK )
            {
                // Get Nunchuck Data
                getNCHUCK();

                sprintf(ans, "%02d,%03d,%03d,%01d,%01d,%03d,%03d,%03d", msgType[index], NCHUCKdata[0], NCHUCKdata[1], NCHUCKdata[2], NCHUCKdata[3], NCHUCKdata[4], NCHUCKdata[5], NCHUCKdata[6]);

                strcat(completeMessage, ans);

                if (DEBUG_MODE)
                    Serial.println("DONE...");

                // Increment index
                index++;
                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }

            if ( msgType[index] == REQUEST_SHTXX )
            {
                if (DEBUG_MODE)
                    Serial.println("REQUEST TYPE > Temperature/humidity...");

                //float temp_c = sht1x.readTemperatureC();
                //float humidity = sht1x.readHumidity(temp_c);
                float temp_c = 0;
                float humidity = 0;


                // Print the values to the serial port
                if(DEBUG_MODE)
                {
                    Serial.print("Temperature: ");
                    Serial.print(temp_c, DEC);
                    Serial.print("C. Humidity: ");
                    Serial.print(humidity);
                    Serial.println("%");
                }

                sprintf(ans, "%d,%d,%d,%d", msgType[index], numberOfSHTXX, (int)(temp_c*100), (int)(humidity*100));

                strcat(completeMessage, ans);

                if (DEBUG_MODE)
                    Serial.println("DONE...");

                // Increment index
                index++;
                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }

            // Add termination to the message
            strcat(completeMessage, end);

            if(DEBUG_MODE)
                Serial.println(completeMessage);

            // Send the message
            Serial.println(completeMessage);

            u1_data_ready = 0;
            memset(u1_rxdata,0,sizeof(u1_rxdata));

            // clear newMessage flag
            newMessage = false;

        }	// end QUERY
        if(mode == STREAM)
        {
            if (DEBUG_MODE)
                Serial.println("STREAM MODE...");

            // Copy the current message			
            memcpy(streamCompleteMessage, completeMessage, sizeof(completeMessage));	

            currentTime = ( millis() - lastTime );

            if(currentTime >= updateRate)
            {
                // Send the message
                Serial.println(completeMessage);

                lastTime = millis();
            }
        }  // end STREAM
        else if(mode == INFO)
        {
            if (DEBUG_MODE)
                Serial.println("INFO MODE...");

            // Save the number of connected sensors
            unsigned char connectedSensors = 0;
            char conStr[32];

            if( activeRANGES == true )
            {
                sprintf(ans, "%d,", REQUEST_RANGES);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeNOSE == true )
            {
                sprintf(ans, "%d,", REQUEST_NOSE);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeTPA81 == true )
            {
                sprintf(ans, "%d,", REQUEST_TPA81);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeANEMOMETER == true )
            {
                sprintf(ans, "%d,", REQUEST_ANEMOMETER);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeSHTXX == true )
            {
                sprintf(ans, "%d,", REQUEST_SHTXX);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            // Add number of sensors
            sprintf(ans, "%d,", connectedSensors);
            strcat(completeMessage, ans);
            // And the sensors
            strcat(completeMessage, conStr);

            // Add termination to the message
            strcat(completeMessage, end);

            // Send the message
            Serial.println(completeMessage);

            // Clear buffer
            memcpy(ans, 0, sizeof(ans));
            memcpy(conStr, 0, sizeof(conStr));


            u1_data_ready = 0;
            memset(u1_rxdata,0,sizeof(u1_rxdata));

        }    // end INFO
        else if(mode == AUTO)
        {
            if (DEBUG_MODE)
                Serial.println("AUTO MODE...");

            int k;

            for( k = 0; k < NUMBER_OF_DIFF_SENSORS; k++ )
            {
                if( sensorInfo[k].numSensors != 0 )
                {
                    if( sensorInfo[k].sensorType == REQUEST_RANGES )
                        activeRANGES = true;
                    else if( sensorInfo[k].sensorType == REQUEST_NOSE )
                        activeNOSE = true;
                    else if( sensorInfo[k].sensorType == REQUEST_TPA81 )
                        activeTPA81 = true;
                    else if( sensorInfo[k].sensorType == REQUEST_ANEMOMETER )
                        activeANEMOMETER = true;

                }
            }

            // Save the number of connected sensors
            unsigned char connectedSensors = 0;
            char conStr[32];

            if( activeRANGES == true )
            {
                sprintf(ans, "%d,", REQUEST_RANGES);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeNOSE == true )
            {
                sprintf(ans, "%d,", REQUEST_NOSE);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeTPA81 == true )
            {
                sprintf(ans, "%d,", REQUEST_TPA81);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeANEMOMETER == true )
            {
                sprintf(ans, "%d,", REQUEST_ANEMOMETER);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            if( activeSHTXX == true )
            {
                sprintf(ans, "%d,", REQUEST_SHTXX);

                strcat(conStr, ans);

                // Increment n sensors
                connectedSensors++;

                // Clear buffer
                memcpy(ans, 0, sizeof(ans));
            }
            // Add number of sensors
            sprintf(ans, "%d,", connectedSensors);
            strcat(completeMessage, ans);
            // And the sensors
            strcat(completeMessage, conStr);

            // Add termination to the message
            strcat(completeMessage, end);

            // Send the message
            Serial.println(completeMessage);

            // Clear buffer
            memcpy(ans, 0, sizeof(ans));
            memcpy(conStr, 0, sizeof(conStr));


            u1_data_ready = 0;
            memset(u1_rxdata,0,sizeof(u1_rxdata));

        }    // end mode AUTO
        else if(mode == VERSION)
        {
            if (DEBUG_MODE)
                Serial.println("VERSION INFO...");

            // Add the current software version
            sprintf(ans, "%d,", ARDUSIM_VERSION);
            strcat(completeMessage, ans);
            
            // Add termination to the message
            strcat(completeMessage, end);

            // Send the message
            Serial.println(completeMessage);

            // Clear buffer
            memcpy(ans, 0, sizeof(ans));
        
            u1_data_ready = 0;
            memset(u1_rxdata,0,sizeof(u1_rxdata));

        }    // end VERSION
        
        
        

    }	// end u1_data_ready

    digitalWrite(LEDpin, HIGH);
}



int getANEM(char addr, char ANEMreg)
{
    if (DEBUG_MODE)
        Serial.println("I2C > Getting ANEM Velocity...");

    return 0;
}




/***********************************
 * Print float numbers to serial
 * can also be sprintf(serial_str, "value:%0d.%d", (int)f, ((f - (int)f) * 100) );
 **********************************/
void serialPrintFloat( float f)
{
    Serial.print((int)f);
    Serial.print(".");
    int temp = (f - (int)f) * 100;
    Serial.println( abs(temp) );
}

