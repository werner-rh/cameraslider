/***
 * Project: BYStepper
 * File   : BYStepper.cpp
 * Author : Werner Riemann 
 * Created: 01.12.2022
 * Board: Arduino Nano
 * 
 * Description: Class definition for controlling the tiny 28BYJ-48 Stepper motor.
 *              The Code is based on Arduino Stepper Lib and stripped down to
 *              4 pin motor. For the 28BYJ-48 pin2 and pin3 of the motor driver has to be
 *              changed. 
 * 
 *              C0 -> IN1
 *              C1 -> IN3
 *              C2 -> IN2
 *              C3 -> IN4
 * 
 * Pins : DirectionPin, StepPin and SleepPin
 * 
 *
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 *
 * The circuits can be found at
 *
 * http://www.arduino.cc/en/Reference/Stepper
 */

// ensure this library description is only included once
#ifndef BYStepper_h
#define BYStepper_h

// library interface description
class BYStepper {

  private:
    void stepMotor(int this_step);

    uint8_t motorRun=0;       // Motor Run/Stop Flag (1 Run, 0 Stop)
    uint8_t direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int stepsPerRevolution;   // total number of steps for one revolution
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;

    unsigned long last_step_time; // time stamp in us of when the last step was taken

    int totalDistanceRotations=0; // count of Rotations (intermal motor) needed for the whole Slider length
    int currentRotationCount=0;  // count of current done rotations from right side started
    unsigned int rotationStepCount=0;

  public:
    // constructors:
    BYStepper(int steps_per_revolution, int motor_pin_1, int motor_pin_2,
                                 int motor_pin_3, int motor_pin_4);
    

    // speed setter method:
    void setSpeed(long whatSpeed);
    void setDirection(uint8_t uDirection);
    void stopMotor();
    void startMotor();
    uint8_t getRunState();

    // mover method:
    void step(int number_of_steps);

    int getRotationCount();
    int getDistanceRotations();
    void setDistanceRotations(int rotations);
    void resetRotationCount();


    int version(void);

  
};

#endif

