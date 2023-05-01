#include <LiquidCrystal.h>

/***
 * Project: Camera Slider motorized
 * File   : CameraSlider.ino
 * Author : Werner Riemann 
 * Created: 01.12.2022
 * Board: Arduino Nano
 * 
 * Description: Steuerung für Camera Slider
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 

 * D2     - Digital in push button 1 - dirLeftButtonPin       - Direction Left, Start, Stop
 * D1     - Digital in push button 0 - dirRightButtonPin      - Direction Right, Start, Stop
 * D3     - Digital in Endswitch left side
 * D4     - Digital in Endswitch right side
 *
 * D5     - Rotary Encoder Switch
 * D6     - Rotary Encoder DT_pin
 * D7     - Rotary Encoder CLK_pin
 * D8     - IN1 Driver ULN2003    -> C0 BYStepper
 * D9     - IN2 Driver ULN2003    -> C2 BYStepper
 * D10    - IN3 Driver ULN2003    -> C1 BYStepper
 * D11    - IN4 Driver ULN2003    -> C3 BYStepper
 * D12    - 
 * D13    - Digital out LEDPIN 13 (interne LED)
 * 
 */

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include "CameraSlider.h"
#include "WRKeyStateDef.h"
#include "BYStepper.h"
#include "CSDisplay.h"

//--- globale Daten, Variablen ----------------------
volatile uint8_t B100HzToggle = 0;  // 100 Hertz Signal
uint8_t ui10MilliSekCount = 0;

int encoder_value = 500;
int last_encoder_value  = 500;
int EncSwitch = 5;
int DT_pin = 6;
int CLK_pin = 7;
int DT_pinstate;
int last_DT_pinstate;

const int stepsPerRevolution = 64;  // change this to fit the number of steps per revolution
int motorSpeed = 60;  // Speed in Percent
uint8_t motorDirection = SLIDE_TORIGHT;   // Motor dreht counterclockwise, Slider fährt nach rechts

//-- Statusvaribalen für Taster ----
volatile  uint8_t encoderBUTTON_State=0;
volatile  uint8_t dirLeft_BUTTON_State=0;
volatile  uint8_t dirRight_BUTTON_State=0;
volatile  uint8_t left_Endswitch_State=0;
volatile  uint8_t right_Endswitch_State=0;
volatile  uint8_t vtUp_Button_State=0;
volatile  uint8_t vtDown_Button_State=0;

CSDisplay display;

// initialize the stepper library on pins 8 through 11:
// C0, C1, C2, C3
BYStepper myStepper(stepsPerRevolution, 8, 10, 9, 11);


// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  ui10MilliSekCount ++;

  if(ui10MilliSekCount >= 10 ) {
    ui10MilliSekCount = 0;
    B100HzToggle ^= 1;
  }

  ReadEncoder();  
}

void setup() {
  pinMode(DIRLEFT_BUTTONPin, INPUT);
  digitalWrite(DIRLEFT_BUTTONPin, HIGH);      // Pullup resistor on
  pinMode(DIRRIGHT_BUTTONPin, INPUT);
  digitalWrite(DIRRIGHT_BUTTONPin, HIGH);      // Pullup resistor on
  pinMode(ENDSWITCH_LEFTPin, INPUT);           // Endswitch Slider left side
  digitalWrite(ENDSWITCH_LEFTPin, HIGH);       
  pinMode(ENDSWITCH_RIGHTPin, INPUT);          // Endswitch Slider right side
  digitalWrite(ENDSWITCH_RIGHTPin, HIGH);


  pinMode (DT_pin, INPUT);
  pinMode (CLK_pin, INPUT);
  digitalWrite(DT_pin, HIGH);
  digitalWrite(CLK_pin, HIGH);
  // Reads the initial state of DT
  last_DT_pinstate = digitalRead(DT_pin);

  display.Setup();
  display.ScreenOut(SCR_WELCOME);
  // set the speed at 60 rpm:
  myStepper.setSpeed(motorSpeed *5);
  myStepper.setDirection(motorDirection);

  readSettings();
  // initialize the serial port:
  //Serial.begin(9600);

  // Timer setup --------------------------
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);  
}

// Einstellungen speichern und lesen --------------------
//This function will write a 2 byte integer to the eeprom at the specified address and address
void EEPROMWriteInt(int p_address, int p_value)
	{
	byte lowByte = ((p_value >> 0) & 0xFF);
	byte highByte = ((p_value >> 8) & 0xFF);

	EEPROM.write(p_address, lowByte);
	EEPROM.write(p_address + 1, highByte);
	}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
	{
	byte lowByte = EEPROM.read(p_address);
	byte highByte = EEPROM.read(p_address + 1);

	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
	}


void saveSettings()
{
  EEPROMWriteInt(0, myStepper.getDistanceRotations());  // Anzahl Rotations für die Sliderstrecke
}

void readSettings()
{
  myStepper.setDistanceRotations(EEPROMReadInt(0));
}

void ReadEncoder() {
    DT_pinstate = digitalRead(DT_pin);
    if (DT_pinstate != last_DT_pinstate) { //did DT changed state?
      if (digitalRead(CLK_pin) == DT_pinstate) { // if DT changed state, check CLK
        encoder_value--; // rotation is counter-clockwise, decrement the value
      }else{
        encoder_value++; // rotation is clockwise, increment the value
      }
    last_DT_pinstate = DT_pinstate; //save the last state of DT
    }
}


/***
 * EncoderValueChange - increments or decrements the value of given var depending
 * from the direction the rotary encoder is moved. The Value of var is kept in range
 * of rangeMin and rangeMax.
 * 
 * param int * valToModify : pointer of var to modify
 * param int rangeMin: minimum value 
 * param int rangeMax: maximum value
*/
void EncoderValueChange(int * valToModify, int rangeMin, int rangeMax) {
    // mit dem Code funktioniert auch schrittweise Änderung um 1
    int aktValue = * valToModify;
    if(last_encoder_value != encoder_value)
    {

      if(encoder_value > last_encoder_value +1)
      {
        last_encoder_value = encoder_value;
        if(aktValue < rangeMax)
          aktValue ++;
      }
      
      if(encoder_value +1  < last_encoder_value )
      {
        last_encoder_value = encoder_value;
        if(aktValue > rangeMin)
        aktValue --;
      }
    }

    * valToModify = aktValue;
}

void CheckDirectionButtons() {

  if(dirLeft_BUTTON_State ==1) // Slider nach links bewegen
  {
    if(myStepper.getRunState() == 1 && motorDirection == SLIDE_TOLEFT)
    {
      myStepper.stopMotor();
      display.clearMoveBar();
    }
    else
    {
      motorDirection = SLIDE_TOLEFT;
      myStepper.setDirection(motorDirection);
      myStepper.startMotor();
      display.showLeftMoveBar();
    }    
  }

  if(dirRight_BUTTON_State ==1) // Slider nach rechts bewegen
  {
    if(myStepper.getRunState() == 1 && motorDirection == SLIDE_TORIGHT)
    {
      myStepper.stopMotor();
      display.clearMoveBar();
    }
    else
    {
      motorDirection = SLIDE_TORIGHT;
      myStepper.setDirection(motorDirection);
      myStepper.startMotor();
      display.showRightMoveBar();
    }
      
  }        
}

void checkMotorSpeedChange(int oldMotorSpeed)
{
  if(motorSpeed != oldMotorSpeed)
  {
    myStepper.setSpeed(motorSpeed *5);
    display.updateSpeed(motorSpeed);
    if(myStepper.getRunState() == 1)
    {
      if(motorDirection == SLIDE_TOLEFT)
        display.showLeftMoveBar();
      if(motorDirection == SLIDE_TORIGHT)
        display.showRightMoveBar();
    }

  }
}

void restoreMoveBar() 
{
if(myStepper.getRunState() == 1)
  {
    if(motorDirection == SLIDE_TOLEFT)
      display.showLeftMoveBar();
    if(motorDirection == SLIDE_TORIGHT)
      display.showRightMoveBar();
  }
}

void clearMoveInfoBar()
{
  myStepper.stopMotor(); 
  display.updateSpeed(0);
  display.clearMoveBar();
  myStepper.setSpeed(motorSpeed *5);  // restore previous speed
}

void setMoveToHome()
{
  myStepper.setSpeed(100 *5);
  display.updateSpeed(100);
  motorDirection = SLIDE_TORIGHT;
  myStepper.setDirection(motorDirection);
  display.showRightMoveBar();
  myStepper.startMotor();
}

void setMoveToLeft()
{
  myStepper.setSpeed(100 *5);
  display.updateSpeed(100);
  motorDirection = SLIDE_TOLEFT;
  myStepper.setDirection(motorDirection);
  display.showLeftMoveBar();
  myStepper.startMotor();
}

void loop() {
  static uint8_t AppState=0;
  static uint8_t StateTrigger = 0;
  uint8_t aktStateTrigger;
  static int aktModeNo = 1;
  static int aktInfoNo = 1;
  static int centerRotatePos = 0;
  static uint8_t homingDone = 0;

  static uint8_t ui100HzSecCounter=0;   // Zähler für den Ablauf einer Sekunde
  
  int oldspeed=0;
  int oldModeNo=0;
  int oldInfoNo=0;

  aktStateTrigger = B100HzToggle;

  // Abarbeitung State-Machine. ----------------------------------------------------------
  // Die State-Machine wird nur bei einem Flankenwechsel, 100 mal je Sekunde, durchlaufen.

  if(aktStateTrigger != StateTrigger) {
    StateTrigger = aktStateTrigger;

    // Taster abfragen
    CheckKeyState(&encoderBUTTON_State, EncSwitch);
    CheckKeyState(&dirLeft_BUTTON_State, DIRLEFT_BUTTONPin);
    CheckKeyState(&dirRight_BUTTON_State, DIRRIGHT_BUTTONPin);

    CheckKeyState(&left_Endswitch_State, ENDSWITCH_LEFTPin);
    CheckKeyState(&right_Endswitch_State, ENDSWITCH_RIGHTPin);
    

    // App-State-Machine abarbeiten und weiterschalten
    switch(AppState) {

      case AST_IDLE:
        if(right_Endswitch_State == 2)  // Slider auf Homeposition nach Einschalten?
          {
            //clearMoveInfoBar();
            myStepper.resetRotationCount();
            homingDone=1;
          }
          
        if(encoderBUTTON_State == 1 || dirLeft_BUTTON_State ==1 || dirRight_BUTTON_State ==1)
        {
        display.ScreenOut(SCR_MODACT_MANUELL);
        display.updateSpeed(motorSpeed);
        AppState = AST_RUN_MANUELL;
        }
        break;

      case AST_RUN_MANUELL:
        if(encoderBUTTON_State == 1)
        {
        //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
        }

        oldspeed = motorSpeed;    
        EncoderValueChange(&motorSpeed, 1, 100);
        checkMotorSpeedChange(oldspeed);

        CheckDirectionButtons();

        if(left_Endswitch_State == 2 && motorDirection == SLIDE_TOLEFT && myStepper.getRunState() == 1)
          myStepper.stopMotor();
        if(right_Endswitch_State == 2 && motorDirection == SLIDE_TORIGHT && myStepper.getRunState() == 1)
          myStepper.stopMotor();
        break;

        case AST_RUN_AUTORET:
          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
            AppState = AST_PREPARE_SELECT;
          }        

          oldspeed = motorSpeed;    
          EncoderValueChange(&motorSpeed, 1, 100);
          checkMotorSpeedChange(oldspeed);

          CheckDirectionButtons();

          if(left_Endswitch_State == 2 && motorDirection == SLIDE_TOLEFT && myStepper.getRunState() == 1)  
          {
            motorDirection = SLIDE_TORIGHT;
            myStepper.setDirection(motorDirection);
            display.showRightMoveBar();
            myStepper.startMotor();
          }        

          if(right_Endswitch_State == 2 && motorDirection == SLIDE_TORIGHT && myStepper.getRunState() == 1)  
          {
            motorDirection = SLIDE_TOLEFT;
            myStepper.setDirection(motorDirection);
            display.showLeftMoveBar();
            myStepper.startMotor();
          }                   
        break;

        case AST_RUN_HOME:
          if(right_Endswitch_State == 2)
          {
            clearMoveInfoBar();

            myStepper.resetRotationCount();
            homingDone=1;
          }

          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
          myStepper.setSpeed(motorSpeed *5);  // restore previous speed
          }       
          break;

        case AST_RUN_LEFT:
          if(left_Endswitch_State == 2)
          {
            clearMoveInfoBar();
          }

          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
          myStepper.setSpeed(motorSpeed *5);  // restore previous speed
          }      
          break;

        case AST_RUN_CENTER:
          if(motorDirection == SLIDE_TOLEFT)
          {
            if(myStepper.getRotationCount() >= centerRotatePos)
            {
              myStepper.stopMotor();
              clearMoveInfoBar();
              AppState = AST_RUN_CENTER_DONE;
            }
          }

          if(motorDirection == SLIDE_TORIGHT)
          {
            if(myStepper.getRotationCount() <= centerRotatePos)
            {
              myStepper.stopMotor();
              clearMoveInfoBar();
              AppState = AST_RUN_CENTER_DONE;
            }
          }

          
          break;

        case AST_RUN_CENTER_DONE:
          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
          myStepper.setSpeed(motorSpeed *5);  // restore previous speed
          }   
          break;

        case AST_RUN_CALIBRATE:
          display.printCalibrateInfo(1,0);
          setMoveToHome();
          AppState = AST_RUN_CALIBRATE_HOME;

          break;

        case AST_RUN_CALIBRATE_HOME:
          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
          myStepper.setSpeed(motorSpeed *5);  // restore previous speed
          }     

          if(right_Endswitch_State == 2) // Slider is Home
          {
            clearMoveInfoBar();
            myStepper.resetRotationCount();
            display.printCalibrateInfo(2,0);
            setMoveToLeft();
            AppState = AST_RUN_CALIBRATE_LEFT;
          }
          break;

        case AST_RUN_CALIBRATE_LEFT:
          if(left_Endswitch_State == 2)
          {
            clearMoveInfoBar();
            myStepper.setDistanceRotations(myStepper.getRotationCount());
            display.printCalibrateInfo(3, myStepper.getDistanceRotations());
            saveSettings();
            AppState = AST_RUN_CALIBRATE_DONE;
          }

          break;

        case AST_RUN_CALIBRATE_DONE:
          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
          myStepper.setSpeed(motorSpeed *5);  // restore previous speed
          }   
          break;

        case AST_RUN_INFO:

          oldInfoNo = aktInfoNo;
          EncoderValueChange(&aktInfoNo, 1, 4);
          if(oldInfoNo != aktInfoNo)
          {
            if(aktInfoNo < 4)
              display.printInfoline(aktInfoNo, myStepper.getDistanceRotations());
            else
              display.printInfoline(aktInfoNo, myStepper.getRotationCount());
          }
          
          if(encoderBUTTON_State == 1)
          {
          //Modus bzw. Menu select
          AppState = AST_PREPARE_SELECT;
          }     
          break;

        case AST_PREPARE_SELECT:
          //Modus bzw. Menu select
          AppState = AST_MODE_SELECT;
          display.ScreenOut(SCR_MENUBASE);
          display.updateModSelect(aktModeNo);
          break;

        case AST_MODE_SELECT:
          oldModeNo = aktModeNo;    
          EncoderValueChange(&aktModeNo, 1, 7);
          if(oldModeNo != aktModeNo)
          {
            display.updateModSelect(aktModeNo);
          }

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_MANUELL)
          {
            display.ScreenOut(SCR_MODACT_MANUELL);
            display.updateSpeed(motorSpeed);
            restoreMoveBar();
            AppState = AST_RUN_MANUELL;
          }

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_AUTORET)
          {
            display.ScreenOut(SCR_MODACT_AUTORET);
            display.updateSpeed(motorSpeed);
            restoreMoveBar();
            AppState = AST_RUN_AUTORET;
          }

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_HOME)
          {
            myStepper.stopMotor();
            display.ScreenOut(SCR_MODACT_HOME);
            setMoveToHome();

            AppState = AST_RUN_HOME;
          }               

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_CENTER && homingDone == 1)
          {
            myStepper.stopMotor();
            display.ScreenOut(SCR_MODACT_CENTER);
            // set move to center
            centerRotatePos = (myStepper.getDistanceRotations() / 2);

            if(centerRotatePos != myStepper.getRotationCount())
            {
              
              if(centerRotatePos > myStepper.getRotationCount())
              {
                setMoveToLeft();
              }
              else
              {
                setMoveToHome();
              }
            }
            AppState = AST_RUN_CENTER;
          }

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_LEFT)
          {
            myStepper.stopMotor();
            display.ScreenOut(SCR_MODACT_LEFT);
            setMoveToLeft();

            AppState = AST_RUN_LEFT;
          }                              

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_CALIBRATE)
          {
            myStepper.stopMotor();
            display.ScreenOut(SCR_MODACT_CALIBRATE);
            

            AppState = AST_RUN_CALIBRATE;
          } 

          if(encoderBUTTON_State == 1 && aktModeNo == AST_RUN_INFO)
          {
            myStepper.stopMotor();
            display.ScreenOut(SCR_MODACT_INFO);
            aktInfoNo = 1;

            AppState = AST_RUN_INFO;
          } 
        break;
    }

  }

  //Serial.println("clockwise");
  if(motorDirection == SLIDE_TOLEFT && left_Endswitch_State !=2 || 
     motorDirection == SLIDE_TORIGHT && right_Endswitch_State !=2)
     {
     myStepper.step(1);
     }
  //delay(500);

  
}

