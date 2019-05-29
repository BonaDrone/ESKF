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

#include <Arduino.h>
#include <Wire.h>
#include <FS.h>

// Drivers for the sensors
#include <VL53L1X.h>
#include <LSM6DSM.h>
#include <PMW3901.h>

uint32_t LOG_TIME = 120 * 1000000;

// data file
File datalog;
uint32_t startTime;
bool shouldLog = true;

// Data collection frequencies in Hz
//float IMU_FREQ = 500.0;
//float ACCEL_FREQ = 100.0;
//float RANGE_FREQ = 75.0;
//float FLOW_FREQ = 100.0;

// Update periods
//float imuUpdatePeriod = 1/IMU_FREQ;
//float accelUpdatePeriod = 1/ACCEL_FREQ;
//float rangeUpdatePeriod = 1/RANGE_FREQ;
//float flowUpdatePeriod = 1/FLOW_FREQ;

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

bool imuRead(float & _ax,float & _ay,float & _az,float & _gx,float & _gy,float & _gz)
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

    // open file to log data
    DOSFS.begin();
    datalog = DOSFS.open("datalog.csv", "w");

    startTime = micros();

    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    digitalWrite(26, LOW);

}

void loop(void)
{
    // Collect data
    // Variables to hold IMU data
    float _ax = 1000.0; // 1000 is an unlikely value to be read from any of the sensors, so it is used to indicate no data
    float _ay = 1000.0;
    float _az = 1000.0;
    float _gx = 1000.0;
    float _gy = 1000.0;
    float _gz = 1000.0;
    // Range data
    float _d = 1000.0;
    // Flow data
    float _fx = 1000.0;
    float _fy = 1000.0;

    uint32_t currentTime = micros();

    // Read sensor data
    bool imuData = imuRead(_ax, _ay, _az, _gx, _gy, _gz);
    bool rangeData = distanceAvailable(_d);

    flowAvailable(_fx, _fy);


    // Collect all the possible data
    if (datalog && shouldLog)
    {
        digitalWrite(25, HIGH);
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
        datalog.print(_fx, 8);
        datalog.print(",");
        datalog.println(_fy, 8);

        if (currentTime - startTime > LOG_TIME) shouldLog = false;

    }
    else
    {
        digitalWrite(26, HIGH);
        digitalWrite(25, LOW);
        if (datalog) datalog.close();
    }

}
