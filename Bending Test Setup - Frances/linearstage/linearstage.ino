/* Example sketch to control a stepper motor with TB6600 stepper motor driver, AccelStepper library and Arduino: acceleration and deceleration. More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include <AccelStepper.h>
#include "AS5600.h"
#include "Wire.h"

AS5600 as5600; // use default wire

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 3  // dir +
#define stepPin 2 // pull + 
#define motorInterfaceType 1

/*---------------State Definitions--------------------------*/
typedef enum
{
  START,
  STOP,
  INC,
  DECR,
  ZERO,
  TEST
} States_t;

States_t state;

// Function declarations
void Execute_Action();

// Starting variables
static char KeyInput = '/';
static int deltaspeed = 10;
static int startspeed = 0;
static int maxspeed = 1000;
int currentspeed = startspeed;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup()
{

  Serial.begin(9600);
  // Set the maximum speed and acceleration:
  // stepper.setAcceleration(0);
  stepper.setSpeed(startspeed);
  stepper.setMaxSpeed(maxspeed);
  state = STOP;

  // set up for the as5600 encoder 
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  Serial.println(as5600.getAddress());

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  delay(1000);
}

void loop()
{
  KeyInput = Serial.read();
  if (KeyInput == 'a')
  {                //
    state = START; // Start moving motor at setup speed
    Serial.println("You hit a");
  }
  else if (KeyInput == 'z')
  {
    state = STOP; // Stop moving the motor
    Serial.println("You hit z");
  }
  else if (KeyInput == 's')
  {
    state = INC; // Increase motor speed
    Serial.println("You hit s");
  }
  else if (KeyInput == 'x')
  {
    state = DECR; // decrease motor speed
    Serial.println("You hit x");
  }
  else if (KeyInput == 'o')
  {
    state = ZERO; // Zero motor speed
    Serial.println("You hit o");
  }
  else if (KeyInput == 't')
  {
    state = TEST;
    Serial.println("You hit test mode");
  }
  else if (KeyInput == 'p')
  {
    // state = TEST;
    stepper.stop();
    stepper.move(-20);
    // stepper.runToPosition(0);
    Serial.println("You took a step");
    state = STOP;
  }
  // KeyInput = '/';
  Execute_Action();
  Serial.print("\tAngle  = ");
  Serial.print(as5600.readAngle());
  Serial.print("\tAngular Speed = ");
  Serial.print(as5600.getAngularSpeed(AS5600_MODE_RPM));
  Serial.print("\tCurrent speed = ");
  Serial.println(currentspeed);
  delay(100);
  // Serial.println(currentspeed);
}

void Execute_Action()
{
  switch (state)
  {
  case START:
    stepper.setSpeed(currentspeed);
    stepper.runSpeed();
    break;
  case STOP:
    stepper.stop();
    break;
  case INC:
    currentspeed = currentspeed + deltaspeed;
    if (currentspeed < 0)
    {
      currentspeed = 0;
    }
    else if (currentspeed > maxspeed)
    {
      currentspeed = maxspeed;
    }
    state = START;
    break;
  case DECR:
    currentspeed = currentspeed - deltaspeed;
    if (abs(currentspeed) < 0)
    {
      currentspeed = 0;
    }
    else if (abs(currentspeed) > maxspeed)
    {
      currentspeed = maxspeed;
    }
    state = START;
    break;
  case ZERO:
    currentspeed = 0;
    state = START;
    break;
  case TEST: // Quick test of motor speed
    // currentspeed = 400;
    // stepper.setSpeed(currentspeed);
    // stepper.runSpeed();
    delay(1000);
    // stepper.stop();
    // state = ZERO;
    break;
  default:
    break;
  }
}
