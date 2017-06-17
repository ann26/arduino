#include <LiquidCrystal.h>

#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temp_celciuserature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas temp_celciuserature.
DallasTemperature sensors(&oneWire);

int RED_LED = 8;
int YELLOW_LED = 9;
// Set the LCD I2C address
// initialize the lcd for 16 chars 2 lines
LiquidCrystal_I2C lcd(0x27, 16, 2);  
// used for keeping track of the last recorded temperature 
float LAST_TEMP = 0.0;
//used for recording highest temp since last reset
float HIGHEST = 0.0;
//used for recording lowest temp since last reset 
float LOWEST = 0.0;
//used for recording average temp since last reset 
float AVERAGE = 0.0;
//used for calculating average
long COUNT = 1;
//used for keeping the sum
float SUM = 0;

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.print("\nThe environment station awakes!\n");
  sensors.begin();
  lcd.begin();  

  // ------- Quick 3 blinks of backlight  -------------
  for (int i = 0; i < 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print("Initialising");
  sensors.requestTemperatures(); 
  float temp_celcius = sensors.getTempCByIndex(0);
  LOWEST = temp_celcius;
  HIGHEST = temp_celcius;
  AVERAGE = temp_celcius;
  SUM = temp_celcius;  
  delay(3000);
  lcd.clear();
}

void loop(void)
{

  // Send the command to get temperature
  sensors.requestTemperatures(); 
  float temp_celcius = sensors.getTempCByIndex(0);

  // short circuit if there is no status change
  if ( temp_celcius == LAST_TEMP)
  {
    // do nothing - avoid doing any other processing to reduce
    // latency
  }
  else
  {
  if (temp_celcius < LOWEST) LOWEST = temp_celcius;
  if (temp_celcius < HIGHEST) HIGHEST = temp_celcius;
  SUM += temp_celcius;
  COUNT += 1;
  AVERAGE = SUM / COUNT;
    update_display(temp_celcius);
  }
  LAST_TEMP = temp_celcius;
  // wait for half a second before polling again
  delay(500);
}

void update_display(float temp_celcius)
{
  lcd.clear();
  lcd.print(temp_celcius);
  lcd.print(" degrees");
  lcd.setCursor(0, 1);  //col, row
  Serial.print(COUNT);
  Serial.print(" reads, modulus is ");
  Serial.print(COUNT % 3);
  Serial.print("\n");
  if ( COUNT % 3 == 0) 
  {
    lcd.print(LOWEST);
    lcd.print(" minimum");
    
  }
  if ( COUNT % 3 == 1) 
  {
    lcd.print(HIGHEST);
    lcd.print(" maximum");
    
  }
  if ( COUNT % 3 == 2) 
  {
    lcd.print(AVERAGE);
    lcd.print(" average");
  }
}
