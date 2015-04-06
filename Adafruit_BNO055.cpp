/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <math.h>
#include <limits.h>

#include "Adafruit_BNO055.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BNO055 class
*/
/**************************************************************************/
Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address)
{
  _sensorID = sensorID;
  _address = address;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode)
{
  /* Enable I2C */
  Wire.begin();

  /* Make sure we have the right device */
  uint8_t id = read8(BNO055_CHIP_ID_ADDR);
  if(id != BNO055_ID)
  {
    return false;
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  
  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);
  
  /* Set the output units */
  uint8_t unitsel = (0 << 7) | /* Orientation = Android */
                    (0 << 4) | /* Temperature = Celsius */
                    (0 << 2) | /* Euler = Degrees */
                    (1 << 1) | /* Gyro = Rads */
                    (0 << 0);  /* Accelerometer = m/s^2 */
  write8(BNO055_UNIT_SEL_ADDR, unitsel);

  /* Orientation = Android (0 << )*/
  /* Temperature = Celsisu (0 << 4)
  uint8_t unitsel = 

  /* Set the requested operating mode (see section 3.3) */
  write8(BNO055_OPR_MODE_ADDR, mode);
  delay(20);

  return true;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode)
{
  _mode = mode;
  
  write8(BNO055_OPR_MODE_ADDR, _mode);
  delay(30);
}

/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void Adafruit_BNO055::getSystemStatus(adafruit_bno055_system_status_t * status)
{
  memset(status, 0, sizeof(adafruit_bno055_system_status_t));
  
  /* Read the system status register */
  status->system_status    = read8(BNO055_SYS_STAT_ADDR);
  status->self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);
  status->system_error     = read8(BNO055_SYS_ERR_ADDR);
}

/**************************************************************************/
/*!
    @brief  Displays system status info via Serial.print
*/
/**************************************************************************/
void Adafruit_BNO055::displaySystemStatus(void)
{
  adafruit_bno055_system_status_t status;
  getSystemStatus(&status);
  
  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */
  
  Serial.print("System Status:          0x");
  Serial.println(status.system_status, HEX);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed
    
     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test
  
     0x0F = all good! */
  
  Serial.print("Self Test Results:      0x");
  Serial.println(status.self_test_result, HEX);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */
  
  Serial.print("System Error:           0x");
  Serial.println(status.system_error, HEX);
}

/**************************************************************************/
/*!
    @brief  Gets the chip revision numbers
*/
/**************************************************************************/
void Adafruit_BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
{
  uint8_t a, b;

  memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

  info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);
  info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);
  info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);
  info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);
  
  a = read8(BNO055_SW_REV_ID_LSB_ADDR);
  b = read8(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/**************************************************************************/
/*!
    @brief  Displays the chip revision numbers via Serial.print
*/
/**************************************************************************/
void Adafruit_BNO055::displayRevInfo(void)
{
  adafruit_bno055_rev_info_t info;
  getRevInfo(&info);

  /* Check the accelerometer revision */
  Serial.print("Accelerometer Revision: 0x");
  Serial.println(info.accel_rev, HEX);
  
  /* Check the magnetometer revision */
  Serial.print("Magnetometer Revision:  0x");
  Serial.println(info.mag_rev, HEX);
  
  /* Check the gyroscope revision */
  Serial.print("Gyroscope Revision:     0x");
  Serial.println(info.gyro_rev, HEX);
  
  /* Check the SW revision */
  Serial.print("SW Revision:            0x");
  Serial.println(info.sw_rev, HEX);
  
  /* Check the bootloader revision */
  Serial.print("Bootloader Revision:    0x");
  Serial.println(info.bl_rev, HEX);
}

/**************************************************************************/
/*!
    @brief  Gets a vector reading from the specified source
*/
/**************************************************************************/
imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type)
{
  imu::Vector<3> xyz;
  uint8_t buffer[6];
  memset (buffer, 0, 6);
  
  int16_t x, y, z;
  x = y = z = 0;
  
  /* Read vector data (6 bytes) */
  readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);
  x = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  y = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  z = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);

  /* Convert the value to an appropriate range (section 3.6.4) */
  /* and assign the value to the Vector type */
  switch(vector_type)
  {
    case VECTOR_MAGNETOMETER:
      /* 1uT = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_GYROSCOPE:
      /* 1rps = 900 LSB */
      xyz[0] = ((double)x)/900.0;
      xyz[1] = ((double)y)/900.0;
      xyz[2] = ((double)z)/900.0;
      break;
    case VECTOR_EULER:
      /* 1 degree = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_ACCELEROMETER:
    case VECTOR_LINEARACCEL:
    case VECTOR_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x)/100.0;
      xyz[1] = ((double)y)/100.0;
      xyz[2] = ((double)z)/100.0;
      break;
  }
  
  return xyz;
}

/**************************************************************************/
/*!
    @brief  Gets a quaternion reading from the specified source
*/
/**************************************************************************/
imu::Quaternion Adafruit_BNO055::getQuat(void)
{
  uint8_t buffer[8];
  memset (buffer, 0, 8);
  
  int16_t x, y, z, w;
  x = y = z = w = 0;
  
  /* Read quat data (8 bytes) */
  readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /* Assign to Quaternion */
  imu::Quaternion quat((double)w, (double)x, (double)y, (double)z);
  return quat;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void Adafruit_BNO055::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "BNO055", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ORIENTATION;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F;
  sensor->min_value   = 0.0F;
  sensor->resolution  = 0.01F;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
bool Adafruit_BNO055::getEvent(sensors_event_t *event)
{
  float orientation;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ORIENTATION;
  event->timestamp = 0;
  /* 
  getPressure(&pressure_kPa);
  event->pressure = pressure_kPa / 100.0F;
  */
  
  return true;
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
bool Adafruit_BNO055::write8(adafruit_bno055_reg_t reg, byte value)
{
  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();

  /* ToDo: Check for error! */
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
byte Adafruit_BNO055::read8(adafruit_bno055_reg_t reg )
{
  byte value = 0;
  
  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(_address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif
  
  return value;
}

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
bool Adafruit_BNO055::readLen(adafruit_bno055_reg_t reg, byte * buffer, uint8_t len)
{
  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(_address, (byte)len);

  /* Wait until data is available */
  while (Wire.available() < len);
    
  for (uint8_t i = 0; i < len; i++)
  {
    #if ARDUINO >= 100
      buffer[i] = Wire.read();
    #else
      buffer[i] = Wire.receive();
    #endif
  }
  
  /* ToDo: Check for errors! */
  return true;
}
