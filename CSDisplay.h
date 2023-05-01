#include <FastIO.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>


/***
 * Project: Camera Slider motorized
 * File   : CSDisplay.h
 * Author : Werner Riemann 
 * Created: 15.12.2022
 * Board: Arduino Nano
 * 
 * Description: Modul for Display outputs
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 
 * 
 */

#include <Arduino.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


// Display Konstanten für ScreenOut
#define SCR_CLEAR       0       // Display löschen
#define SCR_WELCOME     1       // Welcome Message
#define SCR_MENUBASE    10      // Basenumber Menu
#define SCR_MEN_MANUELL 11      // Select Manuell
#define SCR_MEN_AUTORET 12      // Select Auto return
#define SCR_MEN_HOME    13      // Select Home
#define SCR_MEN_CENTER  14
#define SCR_MEN_LEFT    15
#define SCR_MEN_CALIBRATE 16    // Select Calibrate
#define SCR_MEN_INFO      17

#define SCR_MODACT_MANUELL 21   // Action screen Manuell
#define SCR_MODACT_AUTORET 22   // Action screen Auto Return
#define SCR_MODACT_HOME    23   // Action slider Home
#define SCR_MODACT_CENTER  24
#define SCR_MODACT_LEFT    25
#define SCR_MODACT_CALIBRATE 26
#define SCR_MODACT_INFO      27

class CSDisplay
{
private:
    /* data */


public:
    CSDisplay(/* args */);
    void Setup();
    void SetBacklight(uint8_t mode);
    void ScreenOut(uint8_t uiScreenID); 

    void updateSpeed(int ispeed);
    void updateModSelect(uint8_t modeNo);
    void printModeName(uint8_t modeNo);
    void clearMoveBar();
    void showLeftMoveBar();
    void showRightMoveBar();
    void printCalibrateInfo(uint8_t infoNo, int optionalValue);
    void printInfoline(uint8_t infoNo, int optionalValue);
};



