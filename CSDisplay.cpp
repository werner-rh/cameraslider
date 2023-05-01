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
 */

#include "CSDisplay.h"

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);

char szVersion[17] = "[    V0.89     ]";

char szModNames[7][17] = {
  "Manual Slide  ",
  "Auto Return   ",
  "Slider Home   ",
  "Slider Center ",
  "Slider Left   ",
  "Calibration   ",
  "Info WeCaSlide"
  };

CSDisplay::CSDisplay(/* args */)
{
}


void CSDisplay::Setup()
{
  lcd.begin(16, 2);
  lcd.clear();  
}

void CSDisplay::SetBacklight(uint8_t mode)
{
    lcd.setBacklight(mode);
}

void CSDisplay::updateSpeed(int ispeed)
{
char buf[6]="";
  int l=0;
  
  itoa(ispeed, buf, 10);
  l=strlen(buf);
  lcd.setCursor(15-4,1);
  lcd.print("   ");
  lcd.setCursor(15-l,1);
  lcd.print(buf);    
}

/***
 * Prints the selected Modus. 
 * uint8_t modeNo: number of Modus, starting at 1
*/
void CSDisplay::updateModSelect(uint8_t modeNo)
{
  
lcd.setCursor(0,1); 
lcd.print("[");
lcd.print(szModNames[modeNo -1]);
lcd.setCursor(15,1); 
lcd.print("]");
}

void CSDisplay::printModeName(uint8_t modeNo)
{
lcd.setCursor(0,0); 
lcd.print(szModNames[modeNo -1]);
}

void CSDisplay::printCalibrateInfo(uint8_t infoNo, int optionalValue)
{
char szbuf[10];
lcd.setCursor(0,1); 

switch(infoNo)
  {
    case 1:
      lcd.print(szModNames[2]);
      break;

    case 2:
      lcd.print(szModNames[4]);
      break;

    case 3:
      lcd.print("                ");
      lcd.setCursor(0,1); 
      lcd.print("RotSteps: ");
      itoa(optionalValue, szbuf, 10);
      lcd.print(szbuf);
      break;
  }

}

void CSDisplay::printInfoline(uint8_t infoNo, int optionalValue)
{
char szbuf[10];
lcd.setCursor(0,1);

switch (infoNo)
  {
  case 1:
    lcd.print(szVersion);
    break;

  case 2:
      lcd.print("[              ]");
      lcd.setCursor(1,1); 
      lcd.print("RotSteps: ");
      itoa(optionalValue, szbuf, 10);
      lcd.print(szbuf);
    break;

  case 3:
      lcd.print("[              ]");
      lcd.setCursor(1,1); 
      lcd.print("Rail L: 870mm");
      //itoa(optionalValue, szbuf, 10);
      //lcd.print(szbuf);
    break;
  
  case 4:
      lcd.print("[              ]");
      lcd.setCursor(1,1); 
      lcd.print("CurSteps: ");
      itoa(optionalValue, szbuf, 10);
      lcd.print(szbuf);
    break;
  }
}


void CSDisplay::clearMoveBar()
{
  char szBar[2] = "  ";
  lcd.setCursor(14,0);
  lcd.print(szBar);
  lcd.noBlink();
  //lcd.noCursor();
}

void CSDisplay::showLeftMoveBar()
{
  char szBar[2] = "< ";

  lcd.setCursor(14,0);
  lcd.print(szBar);
  lcd.setCursor(15,0);
  //lcd.cursor();
  lcd.blink();
}

void CSDisplay::showRightMoveBar()
{
  char szBar[2] = " >";

  lcd.setCursor(14,0);
  lcd.print(szBar);
  lcd.setCursor(14,0);
  //lcd.cursor();
  lcd.blink();
}

void CSDisplay::ScreenOut(uint8_t uiScreenID)
{
lcd.clear();
lcd.setCursor(0,0); // start ist immer oben links

switch(uiScreenID) 
  {
  case SCR_CLEAR:
    lcd.clear();
    break;

  case SCR_WELCOME:
    lcd.print("-  WeCaSlide   -");
    lcd.setCursor(0,1);
    lcd.print(szVersion);
    lcd.setCursor(4,1);
    break;
  
  case SCR_MENUBASE:
    lcd.print("- Select Modus -");
    break;

  case SCR_MODACT_MANUELL:
    lcd.print(szModNames[0]);
    lcd.setCursor(0,1);
    lcd.print("Speed:         %");
    break;

  case SCR_MODACT_AUTORET:
    lcd.print(szModNames[1]);
    lcd.setCursor(0,1);
    lcd.print("Speed:         %");  
    break;

  case SCR_MODACT_HOME:
    lcd.print(szModNames[2]);
    lcd.setCursor(0,1);
    lcd.print("Speed:         %");  
    break;

  case SCR_MODACT_CENTER:
    lcd.print(szModNames[3]);
    lcd.setCursor(0,1);
    lcd.print("Speed:         %");  
    break;

  case SCR_MODACT_LEFT:
    lcd.print(szModNames[4]);
    lcd.setCursor(0,1);
    lcd.print("Speed:         %");  
    break;

  case SCR_MODACT_CALIBRATE:
    lcd.print(szModNames[5]);
    lcd.setCursor(0,1);
    //lcd.print("Speed:         %");  
    break;

  case SCR_MODACT_INFO:
    lcd.print(szModNames[6]);
    lcd.setCursor(0,1);
    lcd.print(szVersion);
    break;
  }
}
