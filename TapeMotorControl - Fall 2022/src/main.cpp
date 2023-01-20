#include <Arduino.h>
#include <Encoder.h>
#include <PIDControllerV.h>
#include <PIDController.h>
#include <HX711.h>
#include <Teensy_ISR_Timer.h>
#include <TeensyTimerInterrupt.h>

//Pin Definitions--------------------
//Right Spool Pins
#define MOTOR_SR1              9
#define MOTOR_SR2              8
#define MOTOR_SRCONTROL        7
#define ENCODER_SR1            2
#define ENCODER_SR2            1

//Tape Roller Pins
#define MOTOR_RR1              17 
#define MOTOR_RR2              18
#define MOTOR_RRCONTROL        19
#define ENCODER_RR1            15
#define ENCODER_RR2            14

//Left Spool Pins
#define MOTOR_SL1              4
#define MOTOR_SL2              5
#define MOTOR_SLCONTROL        6
#define ENCODER_SL1            23
#define ENCODER_SL2            22

//Pinching Roller Pins
#define MOTOR_PINCH1           12
#define MOTOR_PINCH2           11
#define MOTOR_PINCHCONTROL     10
#define ENCODER_PINCH1         31
#define ENCODER_PINCH2         33

//Moving Pinching Arm Pins
#define MOTORARM1              28
#define MOTORARM2              27
#define MOTORARM_CONTROL       29
#define ENCODER_MA1            26
#define ENCODER_MA2            25

//Right Cable Pins
#define CABLE_R1               39
#define CABLE_R2               38
#define CABLE_RCONTROL         37

//Left Cable Pins
#define CABLE_L1               34
#define CABLE_L2               35
#define CABLE_LCONTROL         36

//Load Cell Inputs
#define loadData_R             40
#define loadClock_R            41
#define loadData_L             16
#define loadClock_L            24
//---------------------------------

#define MAX_SPEED              255

//PID Constants----------------------------------------------------
double vKp = 5;     //Proportional Constant for Velocity Controller
double vKi = .5;    //Integral Constant for Velocity Controller
double vKd = 0;     //Derivative Constant for Velocity Controller
double pKp = 600;   //Proportional Constant for Position Controller
double pKi = 0;     //Integral Constant for Position Controller
double pKd = 0;   //Derivative Constant for Position Controller
//------------------------------------------------------------------

//number of counts per revolution -> 3*986.41
unsigned long int revCountR = 2959;
unsigned long int revCountL = 2959;
unsigned long int revCountroller = 2959;
unsigned long int revCountpinch = 2959;
unsigned long int revCountarm = 2959;
//---------------------------------------------

//Physical constants for rollers, spools, etc------------------------
double startRDiameter = 36;     //Right spool starting tape diameter
double startLDiameter = 36;     //Left spool starting tape diameter
double currentRDiameter = startRDiameter;
double currentLDiameter = startLDiameter;
double rollerDiameter = 15;     //Tape Roller diameter
double pinchDiameter = 14;      //Pinch roller diameter
double tapeThickness = .3;
double speedRatio = 1;
//--------------------------------------------------------------------

//Updating speed/position/tension constants---------------------------
int spoolRSpeed = 0;
int spoolLSpeed = 0;
int rollerSpeed = 0;
int pinchSpeed = 0;
int armSpeed = 0;
int cableRspeed = 0;
int cableLspeed = 0;
float tensionR = 0;
float tensionL = 0;
float tensionValue = 100;   //desired tension in cables (0-500)g
double armPosition = 0;
//----------------------------------------------------------------------

//Interrupt variables---------------------------------------------------
volatile double velocitySR_i = 0;
volatile double prevVelocitySR_i = 0;
volatile long prevTsr_i = 0;
volatile double velocityRR_i = 0;
volatile double prevVelocityRR_i = 0;
volatile long prevTrr_i = 0;
volatile long int encoder_countSR = 0;
volatile long int encoder_countRR = 0;
volatile double velocitySL_i = 0;
volatile double prevVelocitySL_i = 0;
volatile long prevTsl_i = 0;
volatile double velocityPINCH_i = 0;
volatile double prevVelocityPINCH_i = 0;
volatile long prevTpinch_i = 0;
volatile long int encoder_countSL = 0;
volatile long int encoder_countPINCH = 0;
volatile long int encoder_countARM = 0;
//-----------------------------------------------------------------------

//scale calibration------------------------------------------------------
float calibration_factorR = -3125;
float calibration_factorL = -3105;
//-----------------------------------------------------------------------

//User input values and logic--------------------------------------------
bool angleSet = true;
int angleHold = 0;
float speedValue = 0; 
float pinchValue = 0;
float newAngle = 0;
float currAngle = 0;
int currPosition = 0;
char incomingByte;
//------------------------------------------------------------------------

//Class declarations------------------------------------------------------
PIDControllerV spoolRcontroller;
PIDControllerV rollercontroller;
PIDControllerV spoolLcontroller;
PIDControllerV pinchcontroller;
PIDController armcontroller;
PIDController cableRcontroller;
PIDController cableLcontroller;
//HX711 scaleR;
//HX711 scaleL;
IntervalTimer loopTimer;
//------------------------------------------------------------------------

//Functions---------------------------------------------------------------
void setupPins(void);
void encoderSR();
void encoderRR();
void encoderSL();
void encoderPINCH();
void encoderMA();
void tapeControl();
void motor(int spoolRSpeed,int spoolLspeed,int rollerSpeed);
void checkInput(void);
void updateRatio(void);
void printVals(void);
void spoolR(int speed);
void spoolL(int speed);
void roller(int speed);
void pinch(int pinchSpeed);
void arm(int armSpeed);
void cableR(int speed);
void cableL(int speed);
void updateAngle(int angle);
void updateTension(void);
//-------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  setupPins();
  //Interrupt pin setup
  attachInterrupt(digitalPinToInterrupt(ENCODER_SR1),encoderSR,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RR1),encoderRR,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SL1),encoderSL,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINCH1),encoderPINCH,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MA1),encoderMA,RISING);

  //Controller setup
  spoolRcontroller.begin();
  spoolRcontroller.tune(vKp,vKi,vKd);
  spoolRcontroller.limit(-255,255);
  rollercontroller.begin();
  rollercontroller.tune(vKp,vKi,vKd);
  rollercontroller.limit(-255,255);
  spoolLcontroller.begin();
  spoolLcontroller.tune(vKp,vKi,vKd);
  spoolLcontroller.limit(-255,255);
  pinchcontroller.begin();
  pinchcontroller.tune(4,.3,vKd);
  pinchcontroller.limit(-255,255);
  armcontroller.begin();
  armcontroller.tune(pKp,pKi,pKd);
  armcontroller.limit(-255,255);

  //Loosen cables to get accurate zero reading
  cableR(100);
  cableL(64);
  delay(2000);
  cableR(0);
  cableL(0);

  //scale setup
  /*scaleR.begin(loadData_R,loadClock_R);
  scaleR.set_scale();
  scaleR.tare(); //Reset the scale to 0
  scaleR.set_scale(calibration_factorR);
  scaleL.begin(loadData_L,loadClock_L);
  scaleL.set_scale();
  scaleL.tare(); //Reset the scale to 0
  scaleL.set_scale(calibration_factorL);

  //Cable controller setup
  cableRcontroller.begin();
  cableRcontroller.tune(.8,.5,0);//constants need to be tuned
  cableRcontroller.limit(-128,128);
  cableLcontroller.begin();
  cableLcontroller.tune(.8,.5,0);//constants need to be tuned
  cableLcontroller.limit(-128,128);
  Serial.setTimeout(1000);*/

  //3ms interrupt timer to run velocity and position PID controls on higher frequency than scale readings
  loopTimer.begin(tapeControl,2000);
}

void loop() {
  checkInput();
  //updateTension();
  updateRatio(); // spool radius decreases
  // Debugging 
  if (!angleSet || speedValue || pinchValue)
    printVals();
}

//-------------------------------------------------------------------------------------------------------------
/*
tapeControl() runs all tape extrusion, pinching node control, and arm movement. This function is called every
3ms from the interrupt timer. Takes in user input to determined speed of tape extrusion, pinching node, and 
position of pinching arm.
*/
void tapeControl(){
  if(!angleSet)
    updateAngle(newAngle);
  if(!speedValue && !pinchValue){ //speed of extrusion, speed of pinch rollers
    digitalWrite(MOTOR_SR1,HIGH); // blue spool
    digitalWrite(MOTOR_SR2,HIGH);
    digitalWrite(MOTOR_SL1,HIGH); 
    digitalWrite(MOTOR_SL2,HIGH); // gray spool
    digitalWrite(MOTOR_RR1,HIGH); // rolling gears
    digitalWrite(MOTOR_RR2,HIGH);
  } else {
    spoolRcontroller.setpoint((double)(speedValue));
    spoolRSpeed = spoolRcontroller.compute(velocitySR_i);
    rollercontroller.setpoint((double)(-1*speedValue)); // -1 for directionality
    rollerSpeed = rollercontroller.compute(velocityRR_i);
    spoolLcontroller.setpoint((double)(-1*speedValue));
    spoolLSpeed = spoolLcontroller.compute(velocitySL_i);
    pinchcontroller.setpoint((double)(pinchValue));
    pinchSpeed = pinchcontroller.compute(velocityPINCH_i); 
    pinch(pinchSpeed);
    spoolR(spoolRSpeed);
    spoolL(spoolLSpeed);
    roller(rollerSpeed);
  }
  //if (!angleSet || speedValue || pinchValue)
    //printVals();
}

//Configures all pins
void setupPins(void){
  pinMode(MOTOR_SR1,OUTPUT);
  pinMode(MOTOR_SR2,OUTPUT);
  pinMode(MOTOR_RR1,OUTPUT);
  pinMode(MOTOR_RR2,OUTPUT);
  pinMode(MOTOR_SRCONTROL,OUTPUT);
  pinMode(MOTOR_RRCONTROL,OUTPUT);
  pinMode(ENCODER_SR1,INPUT);
  pinMode(ENCODER_SR2,INPUT);
  pinMode(ENCODER_RR1,INPUT);
  pinMode(ENCODER_RR2,INPUT);
  digitalWrite(MOTOR_SR1,HIGH);
  digitalWrite(MOTOR_SR2,HIGH);
  digitalWrite(MOTOR_RR1,HIGH);
  digitalWrite(MOTOR_RR2,HIGH);
  analogWrite(MOTOR_SRCONTROL,0);
  analogWrite(MOTOR_RRCONTROL,0);
  pinMode(MOTOR_SL1,OUTPUT);
  pinMode(MOTOR_SL2,OUTPUT);
  pinMode(MOTOR_PINCH1,OUTPUT);
  pinMode(MOTOR_PINCH2,OUTPUT);
  pinMode(MOTOR_SLCONTROL,OUTPUT);
  pinMode(MOTOR_PINCHCONTROL,OUTPUT);
  pinMode(ENCODER_SL1,INPUT);
  pinMode(ENCODER_SL2,INPUT);
  pinMode(ENCODER_PINCH1,INPUT);
  pinMode(ENCODER_PINCH2,INPUT);
  digitalWrite(MOTOR_SL1,HIGH);
  digitalWrite(MOTOR_SL2,HIGH);
  digitalWrite(MOTOR_PINCH1,HIGH);
  digitalWrite(MOTOR_PINCH2,HIGH);
  analogWrite(MOTOR_SLCONTROL,0);
  analogWrite(MOTOR_PINCHCONTROL,0);
  pinMode(MOTORARM1,OUTPUT);
  pinMode(MOTORARM2,OUTPUT);
  pinMode(MOTORARM_CONTROL,OUTPUT);
  pinMode(ENCODER_MA1,INPUT);
  pinMode(ENCODER_MA2,INPUT);
  digitalWrite(MOTORARM1,HIGH);
  digitalWrite(MOTORARM2,HIGH);
  analogWrite(MOTORARM_CONTROL,0);
  pinMode(CABLE_R1,OUTPUT);
  pinMode(CABLE_R2,OUTPUT);
  pinMode(CABLE_RCONTROL,OUTPUT);
  pinMode(CABLE_L1,OUTPUT);
  pinMode(CABLE_L2,OUTPUT);
  pinMode(CABLE_LCONTROL,OUTPUT);
}

//Encoder functions----------------------------------------------------------------------------
/*
Encoder functions are called everytime, a rising signal is detected on an interrupt pin. In these
funtions, speed and position are calculated which are fed back into the PID controllers for the 
motors which these encoders are attached to. In order to reduce noise, the output velocities are 
constrained to +-75mm/s
*/

void encoderSR(){
  volatile long currTsr = micros();
  if (digitalRead(ENCODER_SR2) == HIGH){ // if ENCODER_B is high increase the count
    encoder_countSR++; // increment the count
    volatile double deltaT = ((double) (currTsr - prevTsr_i))/1.0e6;
    velocitySR_i = 1/deltaT/revCountR*currentRDiameter*PI;  //Calculate speed (mm/s)
    if (velocitySR_i>75.0) //constraining speed
      velocitySR_i = prevVelocitySR_i;
    else
      prevVelocitySR_i = velocitySR_i;
    prevTsr_i = currTsr;
    
  }

  else {// else decrease the count
    encoder_countSR--;  // decrement the count
    volatile double deltaT = ((double) (currTsr - prevTsr_i))/1.0e6;
    velocitySR_i = -1/deltaT/revCountR*currentRDiameter*PI;
    if (velocitySR_i<-75.0)
      velocitySR_i = prevVelocitySR_i;
    else
      prevVelocitySR_i = velocitySR_i;
    prevTsr_i = currTsr;
    //Serial.println(encoder_countSR);
  }
}


void encoderRR(){
  volatile long currTrr = micros();
  if (digitalRead(ENCODER_RR2) == HIGH){ // if ENCODER_B is high increase the count
    encoder_countRR++; // increment the count
    volatile double deltaT = ((double) (currTrr - prevTrr_i))/1.0e6;
    velocityRR_i = 1/deltaT/revCountroller*rollerDiameter*PI;
    prevTrr_i = currTrr;
    if (velocityRR_i>75.0)
      velocityRR_i = prevVelocityRR_i;
    else
      prevVelocityRR_i = velocityRR_i;
  }

  else {// else decrease the count
    encoder_countRR--;  // decrement the count
    volatile double deltaT = ((double) (currTrr - prevTrr_i))/1.0e6;
    velocityRR_i = -1/deltaT/revCountroller*rollerDiameter*PI;
    if (velocityRR_i<-75.0)
      velocityRR_i = prevVelocityRR_i;
    else
      prevVelocityRR_i = velocityRR_i;
    prevTrr_i = currTrr;
}
  
}

void encoderSL(){
  volatile long currTsl = micros();
  if (digitalRead(ENCODER_SL2) == HIGH){ // if ENCODER_B is high increase the count
    encoder_countSL++; // increment the count
    volatile double deltaT = ((double) (currTsl - prevTsl_i))/1.0e6;
    velocitySL_i = 1/deltaT/revCountL*currentLDiameter*PI;
    if (velocitySL_i>75)
      velocitySL_i = prevVelocitySL_i;
    else
      prevVelocitySL_i = velocitySL_i;
    prevTsl_i = currTsl;
    /*Serial.print("F");
    Serial.println(velocitySL_i);*/
  }

  else {// else decrease the count
    encoder_countSL--;  // decrement the count
    volatile double deltaT = ((double) (currTsl - prevTsl_i))/1.0e6;
    velocitySL_i = -1/deltaT/revCountL*currentLDiameter*PI;
    if (velocitySL_i<-75)
      velocitySL_i = prevVelocitySL_i;
    else
      prevVelocitySL_i = velocitySL_i;
    prevTsl_i = currTsl;
    /*Serial.print("B");
    Serial.println(velocitySL_i);*/
  }
}

void encoderPINCH(){
  volatile long currTpinch = micros();
  if (digitalRead(ENCODER_PINCH2) == HIGH){ // if ENCODER_B is high increase the count
    encoder_countPINCH++; // increment the count
    volatile double deltaT = ((double) (currTpinch - prevTpinch_i))/1.0e6;
    velocityPINCH_i = 1/deltaT/revCountpinch*pinchDiameter*PI;
    if (velocityPINCH_i>30.0)
      velocityPINCH_i = prevVelocityPINCH_i;
    else
      prevVelocityPINCH_i = velocityPINCH_i;
    prevTpinch_i = currTpinch;
    /*Serial.print("F");
    Serial.print(pinchSpeed);
    Serial.print("  ");
    Serial.println(velocityPINCH_i);*/
  }

  else {// else decrease the count
    encoder_countPINCH--;  // decrement the count
    volatile double deltaT = ((double) (currTpinch - prevTpinch_i))/1.0e6;
    velocityPINCH_i = -1/deltaT/revCountpinch*rollerDiameter*PI;
    if (velocityPINCH_i<-30.0)
      velocityPINCH_i = prevVelocityPINCH_i;
    else
      prevVelocityPINCH_i = velocityPINCH_i;
    prevTpinch_i = currTpinch;
    /*Serial.print("B");
    Serial.print(pinchSpeed);
    Serial.print("  ");
    Serial.println(velocityPINCH_i);*/
  }
  
}

void encoderMA(){ //moving arm - position controlled to hold specific angle 
  if (digitalRead(ENCODER_MA2) == HIGH) // if ENCODER_B is high increase the count
    encoder_countARM++; // increment the count
  else // else decrease the count
    encoder_countARM--;
}
//---------------------------------------------------------------------------------------------


//Motor functions------------------------------------------------------------------------------
//Motor functions set the speed of the motors determined by the controllers
void spoolR(int speed){
  if (speed > 10) {
    analogWrite(MOTOR_SRCONTROL,speed);
    digitalWrite(MOTOR_SR1, HIGH);
    digitalWrite(MOTOR_SR2, LOW);
  }
  else if (speed < -10) {
    analogWrite(MOTOR_SRCONTROL,abs(speed));
    digitalWrite(MOTOR_SR1, LOW);
    digitalWrite(MOTOR_SR2, HIGH);
  }
  else{
    analogWrite(MOTOR_SRCONTROL,0);  
    digitalWrite(MOTOR_SR1, LOW);
    digitalWrite(MOTOR_SR2, LOW);
  }
}
void spoolL(int speed){
if (speed > 10) {
    analogWrite(MOTOR_SLCONTROL,speed);
    digitalWrite(MOTOR_SL1, HIGH);
    digitalWrite(MOTOR_SL2, LOW);
  }
  else if (speed < -10 ) {
    analogWrite(MOTOR_SLCONTROL,abs(speed));
    digitalWrite(MOTOR_SL1, LOW);
    digitalWrite(MOTOR_SL2, HIGH);
  }
  else{
    analogWrite(MOTOR_SLCONTROL,0);  
    digitalWrite(MOTOR_SL1, LOW);
    digitalWrite(MOTOR_SL2, LOW);
    }
}

void roller(int speed){
  if (speed>10) {
    analogWrite(MOTOR_RRCONTROL,speed);
    digitalWrite(MOTOR_RR1, HIGH);
    digitalWrite(MOTOR_RR2, LOW);
  }
  else if (speed < -10) {
    analogWrite(MOTOR_RRCONTROL,abs(speed));    
    digitalWrite(MOTOR_RR1, LOW);
    digitalWrite(MOTOR_RR2, HIGH);
  }
  else{
    analogWrite(MOTOR_RRCONTROL,0);  
    digitalWrite(MOTOR_RR1, LOW);
    digitalWrite(MOTOR_RR2, LOW);
    }
}

void pinch(int pinchSpeed) {
  if (pinchSpeed>10) {
    analogWrite(MOTOR_PINCHCONTROL,pinchSpeed);
    digitalWrite(MOTOR_PINCH1, HIGH);
    digitalWrite(MOTOR_PINCH2, LOW);
  }
  else if (pinchSpeed <-10) {
    analogWrite(MOTOR_PINCHCONTROL,abs(pinchSpeed));
    digitalWrite(MOTOR_PINCH1, LOW);
    digitalWrite(MOTOR_PINCH2, HIGH);
  }
  else{
    analogWrite(MOTOR_PINCHCONTROL,0);
    digitalWrite(MOTOR_PINCH1, LOW);
    digitalWrite(MOTOR_PINCH2, LOW);
    }
}

void arm(int armSpeed) {
  if (armSpeed>10) {
    analogWrite(MOTORARM_CONTROL,armSpeed);
    digitalWrite(MOTORARM1, HIGH);
    digitalWrite(MOTORARM2, LOW);
  }
  else if (armSpeed <-10) {
    analogWrite(MOTORARM_CONTROL,abs(armSpeed));
    digitalWrite(MOTORARM1, LOW);
    digitalWrite(MOTORARM2, HIGH);
    
  }
  else{
    analogWrite(MOTORARM_CONTROL,0);
    digitalWrite(MOTORARM1, LOW);
    digitalWrite(MOTORARM2, LOW);
    }
}

void cableR(int speed){
  if (speed > 10) {
    analogWrite(CABLE_RCONTROL,speed);
    digitalWrite(CABLE_R1, HIGH);
    digitalWrite(CABLE_R2, LOW);
  }
  else if (speed < -10) {
    analogWrite(CABLE_RCONTROL,abs(speed));
    digitalWrite(CABLE_R1, LOW);
    digitalWrite(CABLE_R2, HIGH);
  }
  else{
    analogWrite(CABLE_RCONTROL,0);  
    digitalWrite(CABLE_R1, LOW);
    digitalWrite(CABLE_R2, LOW);
  }
}

void cableL(int speed){
  if (speed > 10) {
    analogWrite(CABLE_LCONTROL,speed);
    digitalWrite(CABLE_L1, HIGH);
    digitalWrite(CABLE_L2, LOW);
  }
  else if (speed < -10) {
    analogWrite(CABLE_LCONTROL,abs(speed));
    digitalWrite(CABLE_L1, LOW);
    digitalWrite(CABLE_L2, HIGH);
  }
  else{
    analogWrite(CABLE_LCONTROL,0);  
    digitalWrite(CABLE_L1, LOW);
    digitalWrite(CABLE_L2, LOW);
  }
}
//-------------------------------------------------------------

//Update functions---------------------------------------------------------------------
/*
Update functions are called everytime the loop runs through, and are responsible for keeping
track of position of the arm and updating constants (currentRDiameter, currentLDiameter)
*/
void updateRatio(void){
  currentRDiameter = startRDiameter - 2*tapeThickness*encoder_countSR/revCountR;    //Diameter of Right spool tape based on encoder position (mm)
  currentLDiameter = startLDiameter + 2*tapeThickness*encoder_countSL/revCountL;    //Diameter of Left spool tape based on encoder position (mm) - going the other way
  armPosition = double(encoder_countARM)/double(revCountarm)*360;                   //Arm position (deg)
  // revCount - ticks per rev
}

//This function accounts for encoder "drift", and turns off the arm motor when the desired position is reached
void updateAngle(int angle){
  //printVals();
  armcontroller.setpoint((double)(newAngle));     //
  armSpeed = armcontroller.compute(armPosition);
  arm(armSpeed);
  if(abs((armPosition)-(newAngle))<1)  //because the motors stalls, if the speed from the controller is less than 50 for a certain number of times, the arm will turn off
    angleHold++;
  if (angleHold>20){
    digitalWrite(MOTORARM1, HIGH);
    digitalWrite(MOTORARM2, HIGH);
    angleSet = true;
    angleHold = 0;
    //currAngle = armPosition;
    //Serial.println("angle set");
  }
}

//Updates Tension of the cables, keeps it constant
/*void updateTension(void){
  tensionR = scaleR.get_units();
  tensionL = scaleL.get_units();
  if (abs(tensionR)<10)
    cableR(-100);       //if tension reading is low, spool quickly to account for slack
  else{
    cableRcontroller.setpoint((double)(tensionValue));
    cableRspeed = cableRcontroller.compute(tensionR);
    cableR(-1*cableRspeed);
  }
  if (abs(tensionL)<10) //if tension reading is low, spool quickly to account for slack
    cableL(-128);
  else{
  cableLcontroller.setpoint((double)(-1*tensionValue));
  cableLspeed = cableLcontroller.compute(tensionL);
  cableL(cableLspeed);
  }
}*/
//-----------------------------------------------------------------

/*
checkInput() takes user input and sends it to the controllers.
Commands:
'd' + speed     -> speed in mm/s of tape extrusion (+speed = extruding outwards)
'p' + speed     -> speed in mm/s of pinching node movement (+speed = moving towards base)
'a' + angle     -> angle in degrees (+angle  in clockwise direction)
*/
void checkInput(void){
  while (Serial.available()){
    digitalWrite(MOTOR_SR1, HIGH);
    digitalWrite(MOTOR_SR2, HIGH);
    digitalWrite(MOTOR_RR1, HIGH);
    digitalWrite(MOTOR_RR2, HIGH);
    digitalWrite(MOTOR_SL1, HIGH);
    digitalWrite(MOTOR_SL2, HIGH);
    digitalWrite(MOTOR_PINCH1, HIGH);
    digitalWrite(MOTOR_PINCH2, HIGH);
    digitalWrite(MOTORARM1, HIGH);
    digitalWrite(MOTORARM2, HIGH);
    incomingByte = Serial.read(); // stores the /n character
    Serial.println(incomingByte);
    if (incomingByte == 'd') 
        speedValue = Serial.parseFloat(); // stores the integerValue
    if (incomingByte == 'p') // if we receive a newline character we will continue in the loop
        pinchValue = Serial.parseFloat();
    if (incomingByte == 'a'){
        newAngle = Serial.parseFloat();
        //currPosition = armPosition;
        angleSet = false;
    }
    incomingByte = 'b';
    Serial.print(speedValue);
    Serial.print(pinchValue);
    Serial.print(newAngle);
  }
}

//pintvals() used for debugging
void printVals(void){
  Serial.print(speedValue); 
  Serial.print("  ");
  Serial.print(newAngle); 
  Serial.print("  ");
  Serial.print(spoolRSpeed); 
  Serial.print("  ");
  Serial.print(spoolLSpeed);
  Serial.print("  ");
  Serial.print(rollerSpeed); 
  Serial.print("  ");
  Serial.print(pinchSpeed); 
  Serial.print("  ");
  Serial.print(armSpeed);
  Serial.print("  ");
  Serial.print(encoder_countSR); 
  Serial.print("  ");
  Serial.print(encoder_countSL); 
  Serial.print("  ");
  Serial.print(encoder_countRR); 
  Serial.print("  ");
  Serial.print(encoder_countPINCH); 
  Serial.print("  ");
  Serial.print(encoder_countARM); 
  Serial.print("  ");
  Serial.print(velocitySR_i);
  Serial.print("  ");
  Serial.print(velocitySL_i);
  Serial.print("  ");
  Serial.print(velocityRR_i);
  Serial.print("  ");
  Serial.print(velocityPINCH_i);
  Serial.print("  ");
  Serial.print(armPosition); 
  Serial.print("  ");
  Serial.print(abs((armPosition)-(newAngle))); 
  Serial.print("  ");

  Serial.print(currentRDiameter);
  Serial.print("  ");
  Serial.print(currentLDiameter);
  Serial.println("");
}