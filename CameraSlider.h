/***
 * Project: Camera Slider motorized
 * File   : CameraSlider.ino
 * Author : Werner Riemann 
 * Created: 01.12.2022
 * Board: Arduino Nano
 * 
 * Description: Controlling Camera Slider
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 
 * 
 */

// ensure this library description is only included once
#ifndef CameraSlider_h
#define CameraSlider_h

#include <Arduino.h>

#define DIRLEFT_BUTTONPin 2
#define DIRRIGHT_BUTTONPin 1
#define ENDSWITCH_LEFTPin 3
#define ENDSWITCH_RIGHTPin 4

#define SLIDE_TORIGHT 1         // motor counterClockwise, Slider fährt nacht rechts
#define SLIDE_TOLEFT  0         // motor clockwise, Slider fährt nach links

// Defines Statemachine ----------------------------------------------
#define AST_IDLE            0
#define AST_RUN_MANUELL     1
#define AST_RUN_AUTORET     2
#define AST_RUN_HOME        3
#define AST_RUN_CENTER      4
#define AST_RUN_LEFT        5
#define AST_RUN_CALIBRATE   6
#define AST_RUN_INFO        7

#define AST_PREPARE_SELECT  10
#define AST_MODE_SELECT     11

#define AST_RUN_CALIBRATE_HOME 20
#define AST_RUN_CALIBRATE_LEFT 21
#define AST_RUN_CALIBRATE_DONE 22
#define AST_RUN_CENTER_DONE 23




// Prototypen in CameraSlider.ino ------------------------------------
void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);
void saveSettings();
void readSettings();
void ReadEncoder();
void EncoderValueChange(int * valToModify, int rangeMin, int rangeMax);
void CheckDirectionButtons();
void checkMotorSpeedChange(int oldMotorSpeed);
void clearMoveInfoBar();
void restoreMoveBar();
void setMoveToHome();
void setMoveToLeft();

#endif