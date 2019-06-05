/*
   MovePrinterFast.ino : 

   Copyright (c) 2019 Pep Marti Saumell and Juan Gallostra
 */

#include <AccelStepper.h>

// PINS
int const PIN_xSTEP = 54;
int const PIN_xDIR  = 55;
int const PIN_xEN   = 38;
int const PIN_xEND  = 3;

int const PIN_zSTEP = 46;
int const PIN_zDIR  = 48;
int const PIN_zEN   = 63;
int const PIN_zEND  = 18;

// Axis definition
AccelStepper xAxis(AccelStepper::DRIVER, PIN_xSTEP, PIN_xDIR); // pin 3 = step, pin 6 = direction
AccelStepper zAxis(AccelStepper::DRIVER, PIN_zSTEP, PIN_zDIR);

// X Axis Variables
bool  xHoming       = false;
float xMaxSpeed     = 400;  // mm/s
float xGear         = 160.38; //steps/mm
float xHomingSpeed  = 20; //mm/s
int   xHomingDir    = 1;
bool  xEndPressed   = false;
int   xDir          = 1;

// Z Axis Variables
bool  zHoming       = false;
float zMaxSpeed     = 20;  // mm/s
float zGear         = 4266.7; //steps/mm
float zHomingSpeed  = 5; //mm/s
int   zHomingDir    = 1;
bool  zEndPressed   = false;
int   zDir          = 1;
float zPosCurr      = 0.0;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting system...");
  
  // Interrupts
  pinMode(PIN_xEND, INPUT_PULLUP);
  pinMode(PIN_zEND, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_xEND),int_xEndStop, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_zEND),int_zEndStop, RISING);
 
  // Getting switches states
  xEndPressed = digitalRead(PIN_xEND);
  zEndPressed = digitalRead(PIN_zEND);
  xEndPressed = !xEndPressed;
  zEndPressed = !zEndPressed;
  if (zEndPressed)
  {
    Serial.println("Z is pressed");
  }

  // Enabling Motors
  pinMode(PIN_xEN, OUTPUT);
  pinMode(PIN_zEN, OUTPUT);
  digitalWrite(PIN_xEN, LOW);
  digitalWrite(PIN_zEN, LOW);
  
  // Setting Motors parameters
  xAxis.setMaxSpeed(xMaxSpeed*xGear);
  zAxis.setMaxSpeed(zMaxSpeed*zGear);
  xAxis.setAcceleration(2000*xGear);
  zAxis.setAcceleration(2000*zGear);

  delay(5000);
  
}

void loop()
{
    Cycle();
}

int sign(int a)
{
  if (a < 0)
    return -1;

  return 1;
}

void int_xEndStop()
{
  xEndPressed = true;
}

void int_zEndStop()
{
  zEndPressed = true;
  Serial.println("Z pressed");
}

void Cycle(void)
{
    // First stage: Move along x axis at 100mm/s (nearly maximum speed?)
    float xVel = 100;
    for (int ii = 0; ii < 2; ++ii)
    {
        long tStart = millis();
        // x_doCycle goes back and forth along the x axis
        x_doCycle(xVel);
        // Log cycle data to see if we are actually reaching 100mm/s
        long tDuration = millis() - tStart;
        Serial.print("Cycle duration: ");
        Serial.println(tDuration);
        Serial.print("X Velocity: ");
        Serial.println(xVel);
    }


    // second stage, move z up
    zUP();

    // third stage, move x and z at the same time
    xz_doUpCycle();
    
    // Third stage: Move along x axis at 100mm/s (nearly maximum speed?)
    xVel = 100;
    for (int ii = 0; ii < 2; ++ii)
    {
        long tStart = millis();
        // x_doCycle goes back and forth along the x axis
        x_doOpoCycle(xVel);
        // increase velocity for next cycle 
    }

    // Fourth stage
    xz_doDownCycle();

    // Fifth stage
    zDown();
}

void x_doCycle(float vel)
{
  xAxis.moveTo(xGear*-241);
  xAxis.setSpeed(xGear*vel);
  while (xAxis.currentPosition() != xAxis.targetPosition())
  {
    xAxis.runSpeedToPosition();  
  }
  
  // delay to enable some settling time
  delay(3000);
  
  xAxis.moveTo(xGear*-1);
  xAxis.setSpeed(xGear*vel);
  while (xAxis.currentPosition() != xAxis.targetPosition())
  {
    xAxis.runSpeedToPosition();  
  }

  // delay to enable some settling time
  delay(3000);
  
}

void x_doOpoCycle(float vel)
{
  xAxis.moveTo(xGear*-1);
  xAxis.setSpeed(xGear*vel);
  while (xAxis.currentPosition() != xAxis.targetPosition())
  {
    xAxis.runSpeedToPosition();  
  }
  
  // delay to enable some settling time
  delay(3000);
  
  xAxis.moveTo(xGear*-241);
  xAxis.setSpeed(xGear*vel);
  while (xAxis.currentPosition() != xAxis.targetPosition())
  {
    xAxis.runSpeedToPosition();  
  }

  // delay to enable some settling time
  delay(3000);
  
}

void xz_doUpCycle(void)
{
  float vel = 100;
  xAxis.moveTo(xGear*-241);
  xAxis.setSpeed(xGear*vel);

  zAxis.moveTo(zGear*-240/5.0);
  zAxis.setSpeed(zGear*vel/5.0);
  
  while ((xAxis.currentPosition() != xAxis.targetPosition()) && (zAxis.currentPosition() != zAxis.targetPosition()))
  {
    xAxis.runSpeedToPosition();  
    zAxis.runSpeedToPosition();  
  }

  // delay to enable some settling time
  delay(3000);
  
}

void xz_doDownCycle(void)
{
  float vel = 100;
  xAxis.moveTo(xGear*-1);
  xAxis.setSpeed(xGear*vel);

  zAxis.moveTo(zGear*240/5.0);
  zAxis.setSpeed(zGear*vel/5.0);
  
  while ((xAxis.currentPosition() != xAxis.targetPosition()) && (zAxis.currentPosition() != zAxis.targetPosition()))
  {
    xAxis.runSpeedToPosition();  
    zAxis.runSpeedToPosition();  
  }

  // delay to enable some settling time
  delay(3000);
  
}

void zUP(void)
{
  float vel = 20;

  zAxis.moveTo(zGear*-320);
  zAxis.setSpeed(zGear*vel);
  
  while (zAxis.currentPosition() != zAxis.targetPosition())
  {
    zAxis.runSpeedToPosition();  
  }

  // delay to enable some settling time
  delay(3000);
}

void zDown(void)
{
  float vel = 20;

  zAxis.moveTo(zGear*320);
  zAxis.setSpeed(zGear*vel);
  
  while (zAxis.currentPosition() != zAxis.targetPosition())
  {
    zAxis.runSpeedToPosition();  
  }

  // delay to enable some settling time
  delay(3000);
}
