/*
Luggage Drop Sensor Prototype Code For Arduino R3 Breadboarding

Ben Kreimer | 2015
benkreimer.com
benkreimer@gmail.com
*/


#include <Wire.h>

/* Sleep Interrupt Functionality */
#include "LowPower.h"

// Use pin 2 as wake up pin
volatile uint8_t wakePin = 2;
// int ledPin =  13;
volatile uint8_t count = 0;  
volatile uint8_t interruptDone = 0;

#include <ADXL345.h>
ADXL345 adxl;

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
/* Sleep Interrupt Functionality End */


/* DHT Temperature and Humidity Sensor */
#include <DHT22.h>
#include <stdio.h>

#define DHT22_PIN 4  // The digital pin the DHT22 is connected to
DHT22 myDHT22(DHT22_PIN);
/* DHT22 Temperature and Humidity Sensor End */


/* ChronoDot Real Time Clock */
#include "Chronodot.h"
Chronodot RTC;
/* ChronoDot Real Time Clock End */

/* MPU6050 Accelerometer/Gyroscope and Kalman Filter */
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

// MPU6050 Data
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
// double temp; // The MPU6050 contains a temperature sensor
double gyroXangle, gyroYangle; // Angle calculation using the gyroscope
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
/* MPU6050 Accelerometer/Gyroscope and Kalman Filter End */


void wakeUpNow()      
{
 // A handler for the pin interrupt.  
} 


void setup()
{
  // analogReference(EXTERNAL);
// Open serial communications and wait for port to open:
  Serial.begin(9600);
  Wire.begin();
 /* while (!Serial) {
 // ; // wait for serial port to connect. Needed for Leonardo only
  } */
    

/* ChronoDot Real Time Clock */
  RTC.begin();

  if (! RTC.isrunning()) {
   
// The following line sets the RTC to the date & time this sketch was compiled
  RTC.adjust(DateTime(__DATE__, __TIME__));
  }
/* ChronoDot Real Time Clock End */


/* MPU6050 Accelerometer/Gyroscope and Kalman Filter */
TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g, 01=4gs
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

// Set kalman and gyro starting angle
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();
/* MPU6050 Accelerometer/Gyroscope and Kalman Filter End */


/* ADXL 345 Accelerometer */
// Initialise the sensor
  accel.begin();
 
// Set the range to whatever is appropriate for your project
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
  
// Display some basic information about this sensor
  // displaySensorDetails();
  
// Display additional settings (outside the scope of sensor_t)
  // displayDataRate();
  // displayRange();
  // Serial.println("");


// ADXL 345 Interrupt Process
  pinMode(wakePin, INPUT);
  // pinMode(A0, INPUT);
  // pinMode(ledPin, OUTPUT);  
  
  adxl.powerOn();
  // adxl.setRangeSetting(2); //Gs: 2,4,8,16
// Set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(25); // 62.5mg per increment 24 minimum for X axis. Works at 23. This value should remain higher than the inactivity threshold (stock setting: 75).
  adxl.setInactivityThreshold(18); // 62.5mg per increment (stock setting: 75)  Works well at 18.
  adxl.setTimeInactivity(3); // Number of seconds of no activity is inactive? (stock setting: 10).
 
//look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
//look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);  // Must be 1 to avoid incorrect interrupts
 
// Setting all interupts to take place on int pin 1 
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
// adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT2_PIN );
 
// Register interupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
/* ADXL 345 Accelerometer End */


/* DHT22 Temperature and Humidity Sensor */
  DHT22_ERROR_t errorCode;

  errorCode = myDHT22.readData();
  delay(2000);
/* DHT22 Temperature and Humidity Sensor End */
}


void loop()
{
// Interrupt Process  
  // GetInterruptSource clears all triggered actions after returning value
  // so do not call it again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();
  attachInterrupt(0,wakeUpNow, RISING);
  
  
/* Activity Sensed (When the sensor experiences movement) */
  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
    volatile unsigned long starttime;
    volatile unsigned long endtime;
  starttime = millis();
  endtime = starttime;
  while ((endtime - starttime) <=2900) // do this loop for up to 2900mS
{


/* ChronoDot Real Time Clock */
  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  if(now.month() < 10) Serial.print("0");
    Serial.print(now.month(), DEC);
    Serial.print('/');
  if(now.day() < 10) Serial.print("0");
    Serial.print(now.day(), DEC);
    Serial.print(' ');
  if(now.hour() < 10) Serial.print("0");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
  if(now.minute() < 10) Serial.print("0");
    Serial.print(now.minute(), DEC);
    Serial.print(':');
  if(now.second() < 10) Serial.print("0");
    Serial.print(now.second(), DEC);
    Serial.print("|");
/* ChronoDot Real Time Clock End */


/* DHT22 Temperature and Humidity Sensor */
  DHT22_ERROR_t errorCode;

  errorCode = myDHT22.readData();

  Serial.print(myDHT22.getTemperatureC());
  Serial.print("|");
  Serial.print(myDHT22.getHumidity());
/* DHT22 Temperature and Humidity Sensor End */
    
    
/* ADXL 345 Accelerometer */
  sensors_event_t event; 
  accel.getEvent(&event);
// Display the results (acceleration is measured in m/s^2)
  // dataFile.println("Accelerometer:");
  Serial.print("|");
  Serial.print(event.acceleration.x); 
  Serial.print("|");
  Serial.print(event.acceleration.y);
  Serial.print("|"); 
  Serial.print(event.acceleration.z); 
/* ADXL 345 Accelerometer End */
   
      
/* MPU6050 Accelerometer/Gyroscope and Kalman Filter */    
// Update all the values
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
// We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);

  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  timer = micros();

  // temp = ((double)tempRaw + 12412.0) / 340.0;

// Print Data
#if 1 // Set to 1 to activate
  //Serial.print(accX); Serial.print("\t");
  //Serial.print(accY); Serial.print("\t");
  // dataFile.print("Acc Z Raw: "); dataFile.print(accZ); dataFile.print("\t");

  //Serial.print(gyroX); Serial.print("\t");
  //Serial.print(gyroY); Serial.print("\t");
  //Serial.print(gyroZ); Serial.print("\t");
#endif
  // dataFile.print("Acc X Raw: "); dataFile.print(accXangle); dataFile.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  // dataFile.print("Kalman X: "); 
  Serial.print("|");
  Serial.print(kalAngleX); Serial.print("|");

 // dataFile.print("\t");

  // dataFile.print("Acc Y Raw: "); 
  // dataFile.print(accYangle); dataFile.print("|");
  // Serial.print(gyroYangle); Serial.print("\t");
  // Serial.print(compAngleY); Serial.print("\t");
  // dataFile.print("Kalman Y: "); 
  Serial.print(kalAngleY); 
  Serial.print("|");
  // dataFile.print("Z Axis: ");  
  Serial.println(accZ);
  // Serial.print(temp);Serial.print("\t");
  Serial.print("\r\n");
/* MPU6050 Accelerometer/Gyroscope and Kalman Filter End */


  endtime = millis();
}  // While Timer ending bracket
 
  volatile uint8_t interruptDone = 1;
 
  } // Activity interrupt ending bracket
/* Activity Sensed End */


/* Inactivity (The following lines put the sensor to sleep) */
  if (interruptDone == 1) {
    if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
//  digitalWrite(ledPin, LOW);
//  Serial.println("inactivity");
    
  volatile uint8_t interruptDone = 0;
  //add code here to do when inactivity is sensed
//  Serial.println("Timer: Entering Sleep mode");
 
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);     // sleep function called here

}    
  }  
} // voidloop ending bracket
