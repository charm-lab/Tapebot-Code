#include "PIDControllerI.h"

PIDControllerI::PIDControllerI () {
  // Variables - double
  double output;
  double lastErr;
  double lastlastErr;
  double errSum;
  double oldOutput;

  // Variables - long
  unsigned long lastTime;

  // Variables - bool
  bool doConstrain;
  bool init;

  // Variables - double - tuining
  double Kp;
  double Ki;
  double Kd;
  double divisor;
  double minOut;
  double maxOut;
  double setPoint;
}

void PIDControllerI::begin () {
  Kp = 1;
  Ki = 1;
  Kd = 1;
  divisor = 10;
  doLimit = false;
  init = true;
  lastTime = micros();
}

void PIDControllerI::setpoint (double newSetpoint) {
  setPoint = newSetpoint;
}

void PIDControllerI::tune (double _Kp, double _Ki, double _Kd) {
  if (_Kp < 0 || _Ki < 0 || _Kd < 0) return;
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

void PIDControllerI::limit(double min, double max) {
  minOut = min;
  maxOut = max;
  doLimit = true;
}

void PIDControllerI::printGraph (double sensorInput, String verbose) {
  Serial.print(sensorInput);
  if (verbose == VERBOSE) {
    Serial.print(",");
    Serial.print(output);
  }
  Serial.print(",");
  Serial.println(setPoint);
}


void PIDControllerI::minimize (double newMinimize) {
  divisor = newMinimize;
}

// Getters
double PIDControllerI::getOutput () {
  return output;
}


double PIDControllerI::compute (double sensor, String graph, String verbose) {
  // Return false if it could not execute;
  // This is the actual PID algorithm executed every loop();

  // Failsafe, return if the begin() method hasn't been called
  if (!init) return 0;

  // Calculate time difference since last time executed
  unsigned long now = micros();
  double timeChange = (double)(now - lastTime)/1000;

  // Calculate error (P, I and D)
  double error = sensor - setPoint;
  //Serial.println(error);
  errSum = error * timeChange;
  //if (doLimit) {
  //  errSum = constrain(errSum, minOut * 1.1, maxOut * 1.1); 
  //}
  double dErr = (error - 2*lastErr+lastlastErr) / timeChange;
  double change = Kp * (error-lastErr) + Ki * errSum + Kd * dErr;
  //change = constrain(change,-5,5);
  // Calculate the new output by adding all three elements together
  double newOutput = oldOutput+change;
  //Serial.println(newOutput);
  //Serial.println(timeChange);
  // If limit is specifyed, limit the output
  if (doLimit) {
    output = constrain(newOutput, minOut, maxOut);
  } else {
    output = newOutput;  
  }
  //Serial.println(output);

  // Update lastErr and lastTime to current values for use in next execution
  lastlastErr = lastErr;
  lastErr = error;
  lastTime = now;
  oldOutput = output;
  

  // Draw the garph if GRAPH mode
  if (graph == GRAPH) {
    printGraph(sensor, verbose);
  }

  // Return the current output
  return output;
}
