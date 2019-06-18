/*
   CollectDataMotorsOn.ino : Script to collect data from the sensors present in 
   BonaDrone's FC and used to estimate the drone's state. It currently
   collects data from the IMU, the Rangefinder, the Optical Flow and the magnetometer
   at the same time that the motors are running at 50%.

   Additional libraries needed:

       https://github.com/BonaDrone/LSM6DSM
       https://github.com/BonaDrone/LIS2MDL
       https://github.com/BonaDrone/VL53L1X
       https://github.com/BonaDrone/PMW3901

   Hardware support for Bonadrone flight controller:

       https://github.com/BonaDrone/grumpyoldpizza

   Copyright (c) 2019 Juan Gallostra
*/

#include "PMW3901.h"    // Optical Flow
#include "LIS2MDL.h"    // Magnetometer 
#include <LSM6DSM.h>    // IMU
#include <VL53L1X.h>    // Rangefinder

#include <Wire.h>
#include <Arduino.h>


// Using digital pin A4 for chip select
PMW3901 flow(12, &SPI1);

// Motor pins
const uint8_t MOTOR_PINS[4] = {39, 30, 40, 31};

// Magnetometer setup
// These were computed previously by Kris.  
// We use them here to avoid having to calibrate.
static float MAGNETO_BIAS[3]  = {0.814, -0.093, -0.579};
static float MAGNETO_SCALE[3] = {1.11, 1.02, 0.90};

// Specify sensor parameters (sample rate is twice the bandwidth)
// choices are: ODR_10Hz, MOIDR_20Hz, ODR_50 Hz and ODR_100Hz
static LIS2MDL::Rate_t ODR = LIS2MDL::ODR_100Hz;

static LIS2MDL lis2mdl = LIS2MDL(ODR, MAGNETO_BIAS, MAGNETO_SCALE); 

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
    _lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);
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

static void error(const char * msg)
{
    while (true) {
        Serial.print("Error: ");
        Serial.println(msg);
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

// LED pins
const uint8_t blue = 38;
const uint8_t red = 25;

// Collection freqs
int FLOW_FREQ = 25; // Hz
int IMU_FREQ = 250; // Hz
int RANGE_FREQ = 75; // Hz
//int MAGNETO_FREQ = 250; // Hz
uint32_t FLOW_MICROS = 1000000 / FLOW_FREQ;
uint32_t IMU_MICROS = 1000000 / IMU_FREQ;
uint32_t RANGE_MICROS = 1000000 / RANGE_FREQ;
//uint32_t MAGNETO_MICROS = 1000000 / MAGNETO_FREQ;

uint32_t startTime;
uint32_t lastFlowTime;
uint32_t lastIMUTime;
uint32_t lastRangeTime;
uint32_t lastMagnetoTime;

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

    // Start the lis2mdl
    switch (lis2mdl.begin()) {

        case LIS2MDL::ERROR_CONNECT:
            error("no connection");
            break;

        case LIS2MDL::ERROR_ID:
            error("bad ID");
            break;

        case LIS2MDL::ERROR_SELFTEST:
            error("failed self-test");
            break;

         case LIS2MDL::ERROR_NONE:
            break;
    }

    delay(100);
    lis2mdl.clearInterrupt();
    // lis2mdl.calibrate(MAGNETO_BIAS, MAGNETO_SCALE);

    if (!flow.begin()) {
      while(1) { Serial.println("Initialization of the flow sensor failed"); }
    }

    // Start the VL53L1X
    if (!_distanceSensor.begin()) 
    {
      while (true) Serial.println("Range unavailable");
    }

    // init motors
    for (int k=0; k<4; ++k) 
    {
      pinMode(MOTOR_PINS[k], OUTPUT);
      analogWriteFrequency(MOTOR_PINS[k], 10000);  
      analogWrite(MOTOR_PINS[k], 0);  
    }

    delay(3000);

    startTime = micros();
    lastFlowTime = micros();
    lastIMUTime = micros();
    lastMagnetoTime = micros();

    digitalWrite(blue, 0);

}

// Flag that indicates if there is new data
bool newData = false;

bool motorsOff = true;
uint32_t motorsTimer = micros();
// IMU
float _ax, _ay, _az, _gx, _gy, _gz;
// Range
float _d;
// Flow
int16_t deltaX, deltaY;
// Magnetometer
float mx, my, mz;

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t currentTime = micros();

  if (currentTime - motorsTimer > 5*1000000 && motorsOff)
  {
    // Motors at 50% power
    analogWrite(MOTOR_PINS[0], (uint8_t)(0.5 * 255));
    analogWrite(MOTOR_PINS[1], (uint8_t)(0.5 * 255));
    analogWrite(MOTOR_PINS[2], (uint8_t)(0.5 * 255));
    analogWrite(MOTOR_PINS[3], (uint8_t)(0.5 * 255));
    motorsOff = false;
  }
  
  if(currentTime - startTime < 180 * 1000000)
  {
    
    // Read sensor data
    bool imuData = false;
    _ax = 1000.0; // 1000 is an unlikely value to be read from any of the sensors, so it is used to indicate no data
    _ay = 1000.0;
    _az = 1000.0;
    _gx = 1000.0;
    _gy = 1000.0;
    _gz = 1000.0;
    // Magnetometer
    bool magnetoData = false;
    mx = 1000.0;
    my = 1000.0;
    mz = 1000.0;

    // Range
    bool rangeData = false;
    _d = 1000.0;

    // Optical Flow
    bool OpticalData = false;
    deltaX = 1000;
    deltaY = 1000;

    if (currentTime - lastFlowTime > FLOW_MICROS)
    {
        flow.readMotionCount(&deltaX, &deltaY);
        lastFlowTime = currentTime;
        newData = true;
        OpticalData = true;
    }
    
    if (currentTime - lastIMUTime > IMU_MICROS && !OpticalData) 
    {
        imuData = imuRead(_ax, _ay, _az, _gx, _gy, _gz);
        lastIMUTime = currentTime;
        lis2mdl.readData(mx, my, mz);
        magnetoData = true;
        newData = true;
    }

    if (currentTime - lastRangeTime > RANGE_MICROS && !OpticalData)
    {
        rangeData = distanceAvailable(_d);
        lastRangeTime = currentTime;
        newData = true;
    }
//
//    if (currentTime - lastMagnetoTime > MAGNETO_MICROS && !OpticalData)
//    {
//        lis2mdl.readData(mx, my, mz);
//        lastMagnetoTime = currentTime;
//        newData = true;
//        magnetoData = true;
//    }

    if (newData) {
      Serial.print((float)currentTime / 1000000.0, 8);
      Serial.print(",");
      Serial.print(_ax, 8);
      Serial.print(",");
      Serial.print(_ay, 8);
      Serial.print(",");
      Serial.print(_az, 8);
      Serial.print(",");
      Serial.print(_gx, 8);
      Serial.print(",");
      Serial.print(_gy, 8);
      Serial.print(",");
      Serial.print(_gz, 8);
      Serial.print(",");
      Serial.print(mx, 8);
      Serial.print(",");
      Serial.print(my, 8);
      Serial.print(",");
      Serial.print(mz, 8);
      Serial.print(",");
      Serial.print(_d, 8);
      Serial.print(",");
      Serial.print(float(deltaX), 8);
      Serial.print(",");
      Serial.println(float(deltaY), 8);
      newData = false;
    }

  } else {

    // Motors at 0% power
    analogWrite(MOTOR_PINS[0], (uint8_t)(0.0 * 255));
    analogWrite(MOTOR_PINS[1], (uint8_t)(0.0 * 255));
    analogWrite(MOTOR_PINS[2], (uint8_t)(0.0 * 255));
    analogWrite(MOTOR_PINS[3], (uint8_t)(0.0 * 255));
    digitalWrite(blue, 1);
    digitalWrite(red, 0);

  }
  
}
