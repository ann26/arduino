#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"

class Ultrasonic
{
  public:
    Ultrasonic(int pin);
    void DistanceMeasure(void);
    long microsecondsToCentimeters(void);
  private:
    int _pin; //pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
    long duration; // the Pulse time received;
};

Ultrasonic::Ultrasonic(int pin)
{
  _pin = pin;
}

/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_pin, LOW);
  pinMode(_pin, INPUT);
  duration = pulseIn(_pin, HIGH);
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
  return duration / 29 / 2;
}

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temp_celciuserature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas temp_celciuserature.
DallasTemperature sensors(&oneWire);

int RED_LED = 8;
int YELLOW_LED = 9;
// Set the LCD I2C address
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  
Ultrasonic ultrasonic(10);
long BASELINE_DISTANCE = 0.0;
long DISTANCE_CHANGE_COUNT = 0;
//allow for cm variations in distance without triggering
long TOLERANCE = 20; 
// maximum distance sensor can detect in cm
long MAX_DISTANCE = 551;
// used for keeping track of the last recorded temperature 
float LAST_TEMP = 0.0;
// used for keeping track of the last recorded distance
double LAST_DISTANCE = 0;

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  sensors.begin();
  lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight

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
  lcd.setCursor(0, 1);  //col, row
  lcd.print("distance...");
  delay(3000);
  ultrasonic.DistanceMeasure();// get the current signal time;
  BASELINE_DISTANCE = ultrasonic.microsecondsToCentimeters();
  lcd.clear();
  lcd.print("Baseline: ");
  lcd.print(BASELINE_DISTANCE);
  lcd.print("cm");
  delay(3000);
}

void loop(void)
{
  // get the current signal time;
  ultrasonic.DistanceMeasure();
  //convert the time to centimeters
  long range_in_cm = ultrasonic.microsecondsToCentimeters();
  // Send the command to get temperature
  sensors.requestTemperatures(); 
  float temp_celcius = sensors.getTempCByIndex(0);

  // short circuit if there is no status change
  if (( temp_celcius == LAST_TEMP) && ( range_in_cm == LAST_DISTANCE ))
  {
    // do nothing - avoid doing any other processing to reduce
    // latency
  }
  else if (
    ((BASELINE_DISTANCE - TOLERANCE) > range_in_cm) && 
    (range_in_cm < MAX_DISTANCE))
  {
    DISTANCE_CHANGE_COUNT += 1;
    Serial.print("Count: ");
    Serial.println(DISTANCE_CHANGE_COUNT);
    show_movement_leds();
    update_display(temp_celcius, range_in_cm);
  } 
  else
  {
    update_display(temp_celcius, range_in_cm);
    // to reduce latency we will only update temperature leds
    // if there was no movement change
    show_temperature_leds(temp_celcius);
  }
  LAST_TEMP = temp_celcius;
  LAST_DISTANCE = range_in_cm;
}

void update_display(float temp_celcius, double distance_cm)
{
  lcd.clear();
  lcd.print(temp_celcius);
  lcd.print(" degrees");
  lcd.setCursor(0, 1);  //col, row
  lcd.print(DISTANCE_CHANGE_COUNT);
  lcd.print(" (");
  lcd.print(distance_cm);
  lcd.print(" cm)");
}

/* Provide some visual feedback when a movement has been
 *  detected by the distance sensor.
 */
void show_movement_leds()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(RED_LED, HIGH);   
  // turn the LED off (LOW is the voltage level)
  digitalWrite(YELLOW_LED, LOW);   
}

/* Show status indicators using leds if the
 *  temperature has changed between different thresholds.
 */
void show_temperature_leds(float temp_celcius)
{
  float threshold = 30.0; //degrees celcius
  if ( temp_celcius > threshold )
  {
    lcd.backlight();
    digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(YELLOW_LED, LOW);   // turn the LED off (LOW is the voltage level)
  }
  else if (temp_celcius == threshold)
  {
    digitalWrite(RED_LED, HIGH);   // turn the LED off (LOW is the voltage level)
    digitalWrite(YELLOW_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else
  {
    lcd.noBacklight();
    digitalWrite(RED_LED, LOW);   // turn the LED off (LOW is the voltage level)
    digitalWrite(YELLOW_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
}
