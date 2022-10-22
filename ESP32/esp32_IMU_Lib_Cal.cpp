////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib2-Teensy
//
//  Copyright (c) 2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Arduino.h>
#include <Wire.h>
//#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
//#include <I2Cdev.h>
#include "RTIMULib.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"
//#include "ellipsoid_fit.h"

#define ELLIPSOID_FIT_DIR              "/spiffs"

#define ESP32_RTOS  // Uncomment this line if you want to use the code with freertos only on the ESP32
                    // Has to be done before including "OTA.h"

#include "OTA.h"
#include "credentials.h"
#define DBG_STREAM TelnetStream
//#define DBG_STREAM Serial

uint32_t entry;

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object
static RTIMU_DATA imuData;

static RTIMUMagCal *magCal;
static RTIMUAccelCal *accelCal;
static bool magMinMaxDone;
static bool accelEnables[3];
static int accelCurrentAxis;


//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  DBG_STREAM_PORT_SPEED defines the speed to use for the debug DBG_STREAM port

#define  DBG_STREAM_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

void setup()
{
    int errcode;
    printf("vor setupOTA\n");
    setupOTA("TemplateSketch", mySSID, myPASSWORD);
    printf("nach setupOTA\n");

  
    Serial.begin(DBG_STREAM_PORT_SPEED);
    SPIFFS.begin();

//    while (!DBG_STREAM) {
//        ; // wait for DBG_STREAM port to connect. 
//    }
    DBG_STREAM.print("ESP32-IMU\n");
    Wire.begin();
    settings = new RTIMUSettings("/spiffs","RTIMULib");
    imu = RTIMU::createIMU(settings);                        // create the imu object
    DBG_STREAM.print("TeensyIMU starting using device "); DBG_STREAM.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
      DBG_STREAM.print("Failed to init IMU: "); DBG_STREAM.println(errcode);
    }
  
     //  set up for calibration run

    imu->setCompassCalibrationMode(true);
    imu->setAccelCalibrationMode(true);
    DBG_STREAM.printf("RTIMUMagCal\n");
    magCal = new RTIMUMagCal(settings);
    DBG_STREAM.printf("RTIMUMagCal2\n");
    magCal->magCalInit();
    magMinMaxDone = false;
    accelCal = new RTIMUAccelCal(settings);
    accelCal->accelCalInit();
            //doMagMinMaxCal();


    //  the main loop
}
void loop()
{  
    char InputChar;                   // incoming characters stored here
    bool mustExit = false;
  #if !defined(ESP32_RTOS) && !defined(ESP32)
    ArduinoOTA.handle();
  #endif


    if (DBG_STREAM.available()) {
      InputChar = DBG_STREAM.read();
        switch (InputChar) {
        case 'x' :
            mustExit = true;
            break;

        case 'm' :
            doMagMinMaxCal();
            break;

        case 'e' :
            //doMagEllipsoidCal();
            break;

        case 'a' :
            doAccelCal();
            break;
        }
    }

    if(mustExit) {
        DBG_STREAM.printf("\nRTIMULibCal exiting\n");
        return ;
        }
}

void newIMU()
{
    if (imu != NULL)
        delete imu;

    imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        DBG_STREAM.printf("No IMU found\n");
        exit(1);
    }

    //  set up IMU

    imu->IMUInit();
}

void doMagMinMaxCal()
{
    uint64_t displayTimer;
    uint64_t now;
    char input;

    magCal->magCalInit();
    magMinMaxDone = false;

    //  now collect data
    ClrScr();

        CPos(1, 1);


    DBG_STREAM.printf("\n\nMagnetometer min/max calibration\n");
    DBG_STREAM.printf("--------------------------------\n");
    DBG_STREAM.printf("Waggle the IMU chip around, ensuring that all six axes\n");
    DBG_STREAM.printf("(+x, -x, +y, -y and +z, -z) go through their extrema.\n");
    DBG_STREAM.printf("When all extrema have been achieved, enter 's' to save, 'r' to reset\n");
    DBG_STREAM.printf("or 'x' to abort and discard the data.\n");
    DBG_STREAM.printf("\nPress any key to start..\n");
    while(!getUserChar())
    {};

    displayTimer = RTMath::currentUSecsSinceEpoch();

    while (1) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 1000);

        while (pollIMU()) {
            magCal->newMinMaxData(imuData.compass);
            now = RTMath::currentUSecsSinceEpoch();

            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
                displayMagMinMax();
                displayTimer = now;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 's' :
                DBG_STREAM.printf("\nSaving min/max data.\n\n");
                magCal->magCalSaveMinMax();
                magMinMaxDone = true;
                return;

            case 'x' :
                DBG_STREAM.printf("\nAborting.\n");
                return;

            case 'r' :
                DBG_STREAM.printf("\nResetting min/max data.\n");
                magCal->magCalReset();
                break;
            }
        }
    }
}


void doMagEllipsoidCal()
{
    
    uint64_t displayTimer;
    uint64_t now;
    char input;

    if (!magMinMaxDone) {
        DBG_STREAM.printf("\nYou cannot collect ellipsoid data until magnetometer min/max\n");
        DBG_STREAM.printf("calibration has been performed.\n");
        return;
    }

    DBG_STREAM.printf("\n\nMagnetometer ellipsoid calibration\n");
    DBG_STREAM.printf("\n\n----------------------------------\n");
    DBG_STREAM.printf("Move the magnetometer around in as many poses as possible.\n");
    DBG_STREAM.printf("The counts for each of the 8 pose quadrants will be displayed.\n");
    DBG_STREAM.printf("When enough data (%d samples per octant) has been collected,\n", RTIMUCALDEFS_OCTANT_MIN_SAMPLES);
    DBG_STREAM.printf("ellipsoid processing will begin.\n");
    DBG_STREAM.printf("Enter 'x' at any time to abort and discard the data.\n");
    DBG_STREAM.printf("\nPress any key to start...");
    getchar();

    displayTimer = RTMath::currentUSecsSinceEpoch();

    while (1) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 1000);

        while (pollIMU()) {
            magCal->newEllipsoidData(imuData.compass);

            if (magCal->magCalEllipsoidValid()) {
                magCal->magCalSaveRaw(ELLIPSOID_FIT_DIR);
                processEllipsoid();
                return;
            }
            now = RTMath::currentUSecsSinceEpoch();

            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
                displayMagEllipsoid();
                displayTimer = now;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 'x' :
                DBG_STREAM.printf("\nAborting.\n");
                return;
            }
        }
    }
    
}

void processEllipsoid()
{
    
    DBG_STREAM.printf("\n\nProcessing ellipsoid fit data...\n");

    //fit_ellipsoid(RTIMUCALDEFS_MAG_RAW_FILE, RTIMUCALDEFS_MAG_CORR_FILE, 1000);

    magCal->magCalSaveCorr(ELLIPSOID_FIT_DIR);

}

void doAccelCal()
{
    uint64_t displayTimer;
    uint64_t now;
    char input;
        ClrScr();

        CPos(1, 1);


    DBG_STREAM.printf("\n\nAccelerometer Calibration\n");
    DBG_STREAM.printf("-------------------------\n");
    DBG_STREAM.printf("The code normally ignores readings until an axis has been enabled.\n");
    DBG_STREAM.printf("The idea is to orient the IMU near the current extrema (+x, -x, +y, -y, +z, -z)\n");
    DBG_STREAM.printf("and then enable the axis, moving the IMU very gently around to find the\n");
    DBG_STREAM.printf("extreme value. Now disable the axis again so that the IMU can be inverted.\n");
    DBG_STREAM.printf("When the IMU has been inverted, enable the axis again and find the extreme\n");
    DBG_STREAM.printf("point. Disable the axis again and press the space bar to move to the next\n");
    DBG_STREAM.printf("axis and repeat. The software will display the current axis and enable state.\n");
    DBG_STREAM.printf("Available options are:\n");
    DBG_STREAM.printf("  e - enable the current axis.\n");
    DBG_STREAM.printf("  d - disable the current axis.\n");
    DBG_STREAM.printf("  space bar - move to the next axis (x then y then z then x etc.\n");
    DBG_STREAM.printf("  r - reset the current axis (if enabled).\n");
    DBG_STREAM.printf("  s - save the data once all 6 extrema have been collected.\n");
    DBG_STREAM.printf("  x - abort and discard the data.\n");
    DBG_STREAM.printf("\nPress any key to start...\n");
    //while(getUserChar()) {};
    while(!getUserChar())
    {};

    //  perform all axis reset
    for (int i = 0; i < 3; i++)
        accelCal->accelCalEnable(i, true);
    accelCal->accelCalReset();
    for (int i = 0; i < 3; i++)
        accelCal->accelCalEnable(i, false);

    accelCurrentAxis = 0;

    for (int i = 0; i < 3; i++)
        accelEnables[i] = false;

    displayTimer = RTMath::currentUSecsSinceEpoch();

    while (1) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 1000);

        while (pollIMU()) {

            for (int i = 0; i < 3; i++)
                accelCal->accelCalEnable(i, accelEnables[i]);
            accelCal->newAccelCalData(imuData.accel);

            now = RTMath::currentUSecsSinceEpoch();

            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
                displayAccelMinMax();
                displayTimer = now;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 'e' :
                accelEnables[accelCurrentAxis] = true;
                break;

            case 'd' :
                accelEnables[accelCurrentAxis] = false;
                break;

            case 'r' :
                accelCal->accelCalReset();
                break;

            case ' ' :
                if (++accelCurrentAxis == 3)
                    accelCurrentAxis = 0;
                break;

            case 's' :
                accelCal->accelCalSave();
                DBG_STREAM.printf("\nAccelerometer calibration data saved to file.\n");
                return;

            case 'x' :
                DBG_STREAM.printf("\nAborting.\n");
                return;
            }
        }
    }
}


bool pollIMU()
{
    if (imu->IMURead()) {
        imuData = imu->getIMUData();
        return true;
    } else {
        return false;
    }
}

char getUserChar()
{
    char InputChar;                   // incoming characters stored here
    int i;
    if (DBG_STREAM.available()) {
      InputChar = DBG_STREAM.read();
      if(InputChar>13) 
        return tolower(InputChar);
      else
        return 0;
    }
    return 0;
}

void CPos(int x, int y)
{
    DBG_STREAM.printf("\033[%d;%df",y,x);
}

void ClrScr()
{
   DBG_STREAM.printf("\033[2J");
}


void displayMenu()
{
    ClrScr();
    CPos(1, 1);
    DBG_STREAM.printf("\n");
    DBG_STREAM.printf("Options are: \n\n");
    DBG_STREAM.printf("  m - calibrate magnetometer with min/max\n");
    DBG_STREAM.printf("  e - calibrate magnetometer with ellipsoid (do min/max first)\n");
    DBG_STREAM.printf("  a - calibrate accelerometers\n");
    DBG_STREAM.printf("  x - exit\n\n");
    DBG_STREAM.printf("Enter option: ");
}

void displayMagMinMax()
{
    CPos(1, 10);
    //DBG_STREAM.printf("\n\n");
    DBG_STREAM.printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\r\n", magCal->m_magMin.data(0),
           magCal->m_magMin.data(1), magCal->m_magMin.data(2));
    CPos(1, 11);

    DBG_STREAM.printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\r\n", magCal->m_magMax.data(0),
           magCal->m_magMax.data(1), magCal->m_magMax.data(2));
    fflush(stdout);
}

void displayMagEllipsoid()
{
    /*
    int counts[RTIMUCALDEFS_OCTANT_COUNT];

    DBG_STREAM.printf("\n\n");

    magCal->magCalOctantCounts(counts);

    DBG_STREAM.printf("---: %d  +--: %d  -+-: %d  ++-: %d\n", counts[0], counts[1], counts[2], counts[3]);
    DBG_STREAM.printf("--+: %d  +-+: %d  -++: %d  +++: %d\n", counts[4], counts[5], counts[6], counts[7]);
    fflush(stdout);
    */
}

void displayAccelMinMax()
{


        CPos(1, 20);
    DBG_STREAM.printf("Current axis: ");

    if (accelCurrentAxis == 0) {
        DBG_STREAM.printf("x - %s", accelEnables[0] ? "enabled" : "disabled");
    } else     if (accelCurrentAxis == 1) {
        DBG_STREAM.printf("y - %s", accelEnables[1] ? "enabled" : "disabled");
    } else     if (accelCurrentAxis == 2) {
        DBG_STREAM.printf("z - %s", accelEnables[2] ? "enabled" : "disabled");
    }

    CPos(1, 23);
    DBG_STREAM.printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\n", accelCal->m_accelMin.data(0),
           accelCal->m_accelMin.data(1), accelCal->m_accelMin.data(2));
    CPos(1, 24);

    DBG_STREAM.printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", accelCal->m_accelMax.data(0),
           accelCal->m_accelMax.data(1), accelCal->m_accelMax.data(2));
    fflush(stdout);
}
