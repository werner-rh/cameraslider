/***
 * Project: BYStepper - Class to control the tiny 28BYJ-48 Stepper motor
 * File   : BYStepper.cpp
 * Author : Werner Riemann 
 * Created: 01.12.2022
 * Board: Arduino Nano
 * 
 * Description: implements the class
 * 
 * Pins : DirectionPin, StepPin and SleepPin
 * 
 */

#include "Arduino.h"
#include "BYStepper.h"


/*
 *   constructor for four-pin version
 *   Sets which wires should control the motor.
 */
BYStepper::BYStepper(int steps_per_revolution, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 1;      // motor direction - default clockwise
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->stepsPerRevolution = steps_per_revolution; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);
  pinMode(this->motor_pin_3, OUTPUT);
  pinMode(this->motor_pin_4, OUTPUT);

  // pin_count is used by the stepMotor() method:
  this->pin_count = 4;
}



/*
 * Sets the speed in revs per minute
 */
void BYStepper::setSpeed(long speedInRPM)
{
  this->step_delay = 60L * 1000L * 1000L / this->stepsPerRevolution / speedInRPM;
}


/***
 * Sets the rotatet direction 
 */
void BYStepper::setDirection(uint8_t uDirection)
{
  this->direction = uDirection;
}

void BYStepper::stopMotor() 
{
 this->motorRun = 0;
 // all coils to low stops power consumption
 digitalWrite(motor_pin_1, LOW);
 digitalWrite(motor_pin_2, LOW);
 digitalWrite(motor_pin_3, LOW);
 digitalWrite(motor_pin_4, LOW);
}

void BYStepper::startMotor()
{
  this->motorRun = 1;
}

uint8_t BYStepper::getRunState()
{
  return this->motorRun;
}

int BYStepper::getDistanceRotations()
{
  return this->totalDistanceRotations;
}

void BYStepper::setDistanceRotations(int rotations)
{
  this->totalDistanceRotations = rotations;
}

int BYStepper::getRotationCount()
{
  return this->currentRotationCount;
}

void BYStepper::resetRotationCount()
{
  this->currentRotationCount = 0;
  this->rotationStepCount = 0;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void BYStepper::step(int steps_to_move)
{
  // decrement the number of steps, moving one step each time:
  

    unsigned long now = micros();
    // move only if the appropriate delay has passed:
    if (now - this->last_step_time >= this->step_delay && motorRun == 1)
    {
      // step the motor to step number 0, 1, ...,3
      stepMotor(this->step_number);
      // Steps und Rotationen zählen (interne Umdrehungen des Motors ohne Getriebe)
      this->rotationStepCount ++;
      if(this->rotationStepCount >= this->stepsPerRevolution)
      {
        if (this->direction == 0)
          this->currentRotationCount++;
        else
          this->currentRotationCount--;
        this->rotationStepCount = 0;
      }
      
      // get the timeStamp of when you stepped:
      this->last_step_time = now;
      // increment or decrement the step number,
      // depending on direction:
      if (this->direction == 1)  // slide to right
      {
        this->step_number++;
        if (this->step_number > 3) {
          this->step_number = 0;
        }
      }
      else
      {
        this->step_number--;
        if (this->step_number < 0) {
          this->step_number = 3;
        }  
      }
      
    }

}

/*
 * Moves the motor forward or backwards.
 */
void BYStepper::stepMotor(int thisStep)
{
   
    switch (thisStep) {
      case 0:  // 1010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 1:  // 0110
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 2:  //0101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
      case 3:  //1001
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
    }

}

/*
  version() returns the version of the library:
*/
int BYStepper::version(void)
{
  return 5;
}
