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
#include "M5Atom.h"

#define ESP32_RTOS // Uncomment this line if you want to use the code with freertos only on the ESP32
                   // Has to be done before including "OTA.h"

#include "OTA.h"
#include "credentials.h"
#define DBG_STREAM TelnetStream
//#define DBG_STREAM Serial

RTIMU *imu;              // the IMU object
RTIMUSettings *settings; // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL 300 // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define SERIAL_PORT_SPEED 115200

// CAN-Pins
#define CAN_RX_PIN GPIO_NUM_19
#define CAN_TX_PIN GPIO_NUM_22

// I2C-Pins
#define I2C_SDA 26
#define I2C_SCL 32

#define F_GRENZ_HEADING_FILTER 0.1

unsigned long lastDisplay;
unsigned long lastRate;
unsigned long lastIMU;
unsigned long sampleCount;
LED_DisPlay dis;

void setup()
{
    int errcode;
    setupOTA("M5_IMU_9_DOF", mySSID, myPASSWORD);
    // Setup LED
    dis.begin();
    dis.setTaskName("LEDs");
    dis.setTaskPriority(2);
    M5.dis.setCore(1);
    dis.start();

    Serial.begin(SERIAL_PORT_SPEED);
    SPIFFS.begin();

    // while (!Serial) {
    ; // wait for serial port to connect.
    //}

    DBG_STREAM.print("ESP32-IMU\n");
    Wire.begin(I2C_SDA, I2C_SCL, (uint32_t)400000);
    // Wire.begin();

    settings = new RTIMUSettings("/spiffs", "RTIMULib");
    imu = RTIMU::createIMU(settings); // create the imu object
    DBG_STREAM.print("esp32-IMU starting using device ");
    DBG_STREAM.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0)
    {
        DBG_STREAM.print("Failed to init IMU: ");
        DBG_STREAM.println(errcode);
    }

    if (imu->getCompassCalibrationValid())
        DBG_STREAM.println("Using compass calibration");
    else
        DBG_STREAM.println("No valid compass calibration data");

    // set up any fusion parameters here
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    lastDisplay = lastRate = lastIMU = millis();

    sampleCount = 0;
}

void ToggleLed()
{
    static bool led_state = false;
    //  digitalWrite(LED_BUILTIN, led_state);
    led_state = !led_state;
    dis.setBrightness(20);
    if (led_state)
        dis.fillpix(0xff0000); // Illuminate the entire LED lattice with RBG color 0xffffff
    else
        dis.fillpix(0x0000ff); // Illuminate the entire LED lattice with RBG color 0xffffff
}

double PT_1funk(double f_grenz, double t_abtast, double oldvalue, double newvalue)
{

    double T = 1 / (2 * PI * f_grenz);
    double tau = 1.0 / ((T / t_abtast) + 1.0);
    return (oldvalue + tau * (newvalue - oldvalue));
}

void ClrScr()
{
    DBG_STREAM.printf("\033[2J");
}

void CPos(int x, int y)
{
    DBG_STREAM.printf("\033[%d;%df", y, x);
}

RTQuaternion rotvecquat(RTVector3 v, RTQuaternion q) //  take a rotation-vector and quaternion, and rotate the vector by the quaternion
{
    RTQuaternion w(0, v.x(), v.y(), v.z()), r(q.scalar(), -q.x(), -q.y(), -q.z());
    return ((q * w) * r);
}

void loop()
{
    unsigned long now = millis();
    unsigned long dt;
    unsigned long delta;
    RTIMU_DATA imuData;
    static double heading_max = 0, heading_min = 720, heading_old = 0, filtered_heading = 750;
    static int cc = 0;
    static double rot_sum = 0, frot_sum = 0;
    double heading, rot, frot;

#if !defined(ESP32_RTOS) && !defined(ESP32)
    ArduinoOTA.handle();
#endif

    if (imu->IMURead())
    { // get the latest data if ready yet
        imuData = imu->getIMUData();
        dt = now - lastIMU;
        lastIMU = now;

        heading = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
        heading = heading < 0 ? 360.0 + heading : heading;
        if (filtered_heading > 720)
            filtered_heading = heading; // startwert für pt1

        filtered_heading = PT_1funk(F_GRENZ_HEADING_FILTER, dt / 1000.0, filtered_heading, heading);

        sampleCount++;
        if ((delta = now - lastRate) >= 1000)
        {
            CPos(1, 10);
            DBG_STREAM.print("Sample rate: ");
            DBG_STREAM.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                DBG_STREAM.print(", gyro bias valid\n");
            else
                DBG_STREAM.print(", calculating gyro bias\n");

            if (!imu->getCompassCalibrationValid())
            {
                DBG_STREAM.println("No valid compass calibration data");
                if (imu->getRuntimeCompassCalibrationValid())
                    DBG_STREAM.print(", runtime mag cal valid\n");
                else
                    DBG_STREAM.print(", runtime mag cal not valid\n");
            }
            else
                DBG_STREAM.println("Using compass calibration");

            DBG_STREAM.println();
            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL)
        {
            ToggleLed();

            lastDisplay = now;
            CPos(1, 20);
            DBG_STREAM.print(RTMath::displayRadians("Gyro:", imuData.gyro)); // gyro data
            CPos(1, 21);
            DBG_STREAM.print(RTMath::displayRadians("Accel:", imuData.accel)); // accel data
            CPos(1, 22);
            DBG_STREAM.print(RTMath::displayRadians("Mag:", imuData.compass)); // compass data
            CPos(1, 23);
            DBG_STREAM.print(RTMath::displayDegrees("Pose:", imuData.fusionPose)); // fused output
            Serial.print(RTMath::displayDegrees("Pose:", imuData.fusionPose));     // fused output
            Serial.println();
            CPos(1, 24);
            DBG_STREAM.print(RTMath::display("QPose:", imuData.fusionQPose)); // RTQuaternion
            DBG_STREAM.println();
            /*
                        heading=imuData.fusionPose.z()*180/PI;
                        heading = heading < 0 ? 360.0 + heading : heading;
                        if(filtered_heading>720)
                            filtered_heading=heading;   // startwert für pt1
                        filtered_heading = PT_1funk(0.1, 0.3, filtered_heading, heading);
            */
            rot = imuData.gyro.z() * RTMATH_RAD_TO_DEGREE;
            if (cc > 20)
            {
                heading_max = filtered_heading > heading_max ? filtered_heading : heading_max;
                heading_min = filtered_heading < heading_min ? filtered_heading : heading_min;
            }
            RTQuaternion gyroq = rotvecquat(imuData.gyro, imuData.fusionQPose);
            frot = gyroq.z() * RTMATH_RAD_TO_DEGREE; // korrigierter rotations-vektor
            CPos(1, 25);
            DBG_STREAM.print(RTMath::display("Rotvec:", gyroq)); // RTQuaternion

            // frot=gyroq.z()*180/PI;
            CPos(1, 27);
            rot_sum += rot;
            frot_sum += frot;
            DBG_STREAM.printf("Heading: %5.2lf\t Headingfilt: %5.2lf\t dHeading: %5.2lf\t ,rot\t %3.1lf, frot\t %3.1lf °/s, rotsum: %lf, frotsum: %lf\n", heading, filtered_heading, heading_max - heading_min, rot, frot, rot_sum, frot_sum);                                                                    // fused output
            Serial.printf("Heading: % 5.2lf,\t Headingfilt: % 5.2lf, Hmax: % 5.2lf, Hmin: % 5.2lf,\t dHeading: % 5.2lf,\t ,rot:\t % 3.1lf, frot:\t % 3.1lf, °/s, rotsum: % lf,, frotsum: % lf,\n", heading, filtered_heading, heading_max, heading_min, heading_max - heading_min, rot, frot, rot_sum, frot_sum); // fused output
            CPos(1, 30);

            cc++;

            // Serial.println();
        }
    }
}
