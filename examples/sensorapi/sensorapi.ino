#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
    
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Displays some basic information on the BNO055 HW
*/
/**************************************************************************/
void displayChipDetails(void)
{
  /* Chip Revision */
  Adafruit_BNO055::adafruit_bno055_rev_info_t revInfo;
  bno.getRevInfo(&revInfo);
  Serial.println("------------------------------------");
  Serial.print  ("Accel Rev:    0x"); Serial.println(revInfo.accel_rev, HEX);
  Serial.print  ("Mag Rev:      0x"); Serial.println(revInfo.mag_rev, HEX);
  Serial.print  ("Gyro Rev:     0x"); Serial.println(revInfo.gyro_rev, HEX);
  Serial.print  ("SW Rev:       0x"); Serial.println(revInfo.sw_rev, HEX);
  Serial.print  ("Boot Rev:     0x"); Serial.println(revInfo.bl_rev, HEX);
  Serial.println("------------------------------------");
  Serial.println("");
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Display some basic information on this sensor */
  displayChipDetails();
  
  /* Calibration info warning */
  /* Move the device in a figure 8 motion to generate mag cal data */
  Serial.println(F("* indicates calibrated data"));
  Serial.println(F("! indicates uncalibrated data"));
  Serial.println(F(""));

  bno.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Board layout:
       +----------+
       |o    |   o|          ______________         Z
   VIN |*    |    |         / *           /|        ^  X
   3Vo |*        *| PS0    /             / /        | /
   GND |*  *--   *| PS1   /             / /         |/
   SDA |*  ---   *| INT   -------------- /     Y <--+
   SCL |*        *| ADR   --------------
   RST |*         |           BNO055
       |o        o|
       +----------+

   Roll:    Rotation around the Y axis (-90° <= roll <= 90°)
            Positive values increasing when X moves towards Z
   Pitch:   Rotation around the X axis (-180° <= pitch <= 180°C)
            Positive values increasing when Z moves towards Y
   Heading: Rotation around the Z axis (0° <= Heading < 360°)
            North = 0°, East = 90°, South = 180°, West = 270°
  */
    
  /* Display the floating point data */
  Serial.print("Roll: ");
  Serial.print(event.orientation.roll);
  Serial.print("\tPitch: ");
  Serial.print(event.orientation.pitch);
  Serial.print("\tHeading: ");
  Serial.print(event.orientation.heading);
  Serial.print("");

  /* Make sure the magnetometer is fully calibrated (bits 4..5 = 11) */  
  uint8_t calStatus = bno.getCalStatus();
  if (calStatus && 0x30)
  {
    /* Data comes from calibrated sensor */
    Serial.println(F("\t*"));
  }
  else
  {
    /* Sensor is not fully calibrated ... move in figure-8 motion */
    Serial.println(F("\t!"));
  }
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}