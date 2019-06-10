/*
   CollectData.ino : Script to collect data from the sensors present in 
   BonaDrone's FC and used to estimate the drone's status. It currently
   collects data from the IMU, the Rangefinder and the Optical Flow.

   This script runs, on BonaDrone's FC, at about 172Hz.

   Additional libraries needed:

       https://github.com/BonaDrone/LSM6DSM
       https://github.com/BonaDrone/VL53L1X
       https://github.com/BonaDrone/PMW3901

   Hardware support for Bonadrone flight controller:

       https://github.com/BonaDrone/grumpyoldpizza

   Copyright (c) 2019 Juan Gallostra
 */

#include "PMW3901.h"
#include <LSM6DSM.h>
#include <VL53L1X.h>

#include <Wire.h>
#include <FS.h>
#include <Arduino.h>


File datalog;
// Using digital pin A4 for chip select
PMW3901 flow(12, &SPI1);

// IMU setup
// LSM6DSM data-ready interrupt pin
const uint8_t LSM6DSM_INTERRUPT_PIN = 2;
// LSM6DSM settings. Options are:
/*
    Ascale_t: AFS_2G, AFS_16G, AFS_4G, AFS_8G
    Gscale_t: GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    Rate_t: ODR_12_5Hz, ODR_26Hz, ODR_52Hz, ODR_104Hz, ODR_208Hz, ODR_416Hz, ODR_833Hz, ODR_1660Hz, ODR_3330Hz, ODR_6660Hz
*/
static const LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_4G;
static const LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_2000DPS;
static const LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_416Hz;
static const LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_416Hz;
// Gyro bias will be estimated by the ESKF filter
float ACCEL_BIAS[3] = {0.0,0.0,0.0};
float GYRO_BIAS[3]  = {0.0,0.0,0.0};
// IMU instance
LSM6DSM _lsm6dsm = LSM6DSM(Ascale, Gscale, AODR, GODR, ACCEL_BIAS, GYRO_BIAS);

bool imuRead(float & _ax, float & _ay, float & _az, float & _gx, float & _gy, float & _gz)
{
    if (_lsm6dsm.checkNewData()) 
    {
        _lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);
        return true;
    } 
    return false;
}

void i2cerror(const char * devicename)
{
    while (true) 
    {
        Serial.print("Unable to start: ");
        Serial.println(devicename);
    }
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

const uint8_t blue = 38;
const uint8_t red = 25;

// Collection freqs
int FLOW_FREQ = 25; // Hz
int ACCEL_FREQ = 100; // Hz
uint32_t FLOW_MICROS = 1000000 / FLOW_FREQ;
uint32_t ACCEL_MICROS = 1000000 / ACCEL_FREQ;

uint32_t startTime;
uint32_t lastFlowTime;
uint32_t lastAccelTime;

//uint16_t FLOW_OUTLIER = 30;


void setup() {
  
    pinMode(blue, OUTPUT);
    digitalWrite(blue, 1);
    pinMode(red, OUTPUT);
    digitalWrite(red, 1);
    
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


    if (!flow.begin()) {
      while(1) { Serial.println("Initialization of the flow sensor failed"); }
    }

    // Start the VL53L1X
    if (!_distanceSensor.begin()) 
    {
      while (true) Serial.println("Range unavailable");
    }

    DOSFS.begin();
    datalog = DOSFS.open("datalog.csv", "w");

    delay(3000);

    startTime = micros();
    lastFlowTime = micros();
    lastAccelTime = micros();

}

void loop() {

  uint32_t currentTime = micros();
  
  if(currentTime - startTime < 30 * 1000000)
  {
    
    digitalWrite(blue, 0);
    
    // Read sensor data
    // IMU
    float _ax, _ay, _az, _gx, _gy, _gz;
    _ax = 1000.0; // 1000 is an unlikely value to be read from any of the sensors, so it is used to indicate no data
    _ay = 1000.0;
    _az = 1000.0;
    _gx = 1000.0;
    _gy = 1000.0;
    _gz = 1000.0;

    bool accelData = false;
    bool imuData = false;

    // Since we collect IMU data at max speed and accel and 
    // IMU data come from the same source, we prioritize collecting
    // accel data when required. Otherwise we would never collect
    // accel data.
    if (currentTime - lastAccelTime > ACCEL_MICROS)
    {
        accelData = imuRead(_ax, _ay, _az, _gx, _gy, _gz);
        _gx = 1000.0;
        _gy = 1000.0;
        _gz = 1000.0;
        lastAccelTime = currentTime;
    } else {
        imuData = imuRead(_ax, _ay, _az, _gx, _gy, _gz);
    }


    // Range
    float _d;
    _d = 1000.0;

    bool rangeData = distanceAvailable(_d);

    // Optical Flow
    int16_t deltaX = 1000, deltaY = 1000;
    if (currentTime - lastFlowTime > FLOW_MICROS)
    {
      flow.readMotionCount(&deltaX, &deltaY);
      lastFlowTime = currentTime;
    }
    
    datalog.print((float)currentTime / 1000000.0, 8);
    datalog.print(",");
    datalog.print(_ax, 8);
    datalog.print(",");
    datalog.print(_ay, 8);
    datalog.print(",");
    datalog.print(_az, 8);
    datalog.print(",");
    datalog.print(_gx, 8);
    datalog.print(",");
    datalog.print(_gy, 8);
    datalog.print(",");
    datalog.print(_gz, 8);
    datalog.print(",");
    datalog.print(_d, 8);
    datalog.print(",");
    datalog.print(float(deltaX), 8);
    datalog.print(",");
    datalog.println(float(deltaY), 8);

  } else {

    digitalWrite(blue, 1);
    digitalWrite(red, 0);
    datalog.close();

  }
}
