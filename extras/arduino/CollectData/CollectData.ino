/*
   CollectData.ino : 

   Additional libraries needed:

       https://github.com/simondlevy/LSM6DSM
       https://github.com/simondlevy/VL53L1X

   Hardware support for Bonadrone flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Juan Gallostra
 */

#include <Arduino.h>
#include <Wire.h>

// Drivers for the sensors
#include <VL53L1X.h>
#include <LSM6DSM.h>
#include <PMW3901.h>

// IMU setup
// LSM6DSM data-ready interrupt pin
const uint8_t LSM6DSM_INTERRUPT_PIN = 2;
// LSM6DSM full-scale settings
static const LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_4G;
static const LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_2000DPS;
static const LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_833Hz;
static const LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_833Hz;
// Gyro bias will be estimated by the ESKF filter
float ACCEL_BIAS[3] = {0.0,0.0,0.0};
float GYRO_BIAS[3]  = {0.0,0.0,0.0};
// IMU instance
LSM6DSM _lsm6dsm = LSM6DSM(Ascale, Gscale, AODR, GODR, ACCEL_BIAS, GYRO_BIAS);
// Variables to hold IMU data
float _ax = 0;
float _ay = 0;
float _az = 0;
float _gx = 0;
float _gy = 0;
float _gz = 0;

bool imuRead(void)
{
    if (_lsm6dsm.checkNewData()) 
    {
        _lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);
        return true;
    } 
    return false;
}

// Rangefinder setup
VL53L1X _distanceSensor;

bool distanceAvailable(float & distance)
{
    if (_distanceSensor.newDataReady()) 
    {
        distance = _distanceSensor.getDistance() / 1000.f; // mm => m
        return true;
    }
    return false;
}

// Optical flow setup
// Use digital pin 12 for chip select and SPI1 port for comms
PMW3901 _flowSensor = PMW3901(12, &SPI1);
float _deltaX = 0;
float _deltaY = 0;

bool flowAvailable(float & deltaX, float & deltaY)
{
    int16_t _deltaX=0, _deltaY=0;
    _flowSensor.readMotionCount(&_deltaX, &_deltaY);
    deltaX = (float)_deltaX;
    deltaY = (float)_deltaY;
    return true; 
}

void i2cerror(const char * devicename)
{
    while (true) 
    {
        Serial.print("Unable to start: ");
        Serial.println(devicename);
    }
}

void setup(void)
{
    // begin the serial port
    Serial.begin(115200);

    // IMU setup
    // Configure interrupt
    pinMode(LSM6DSM_INTERRUPT_PIN, INPUT);

    // Start I^2C
    Wire.begin(TWI_PINS_20_21);
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(100);

    // Start the LSM6DSM
    switch (_lsm6dsm.begin()) 
    {
      case LSM6DSM::ERROR_CONNECT:
        i2cerror("no connection");
        break;
      case LSM6DSM::ERROR_ID:
        i2cerror("bad ID");
        break;
      case LSM6DSM::ERROR_SELFTEST:
        //i2cerror("failed self-test");
        break;

      case LSM6DSM::ERROR_NONE:
        break;
    }
    delay(100);
    // Clear the interrupt
    _lsm6dsm.clearInterrupt();
    _lsm6dsm.calibrate(GYRO_BIAS, ACCEL_BIAS, 127);

    // Start the VL53L1X
    if (!_distanceSensor.begin()) 
    {
       while (true) Serial.println("Range unavailable");
    }

    // Start the PMW3901
    if (!_flowSensor.begin()) {
       while (true) Serial.println("Flow unavailable");
    }
}

void loop(void)
{

}
