/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> https://www.adafruit.com/product/2472

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

  // BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  Wire.setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  /* Make sure we have the right device */
  uint8_t id = read8(BNO055_CHIP_ID_ADDR);
  if(id != BNO055_ID)
  {
    delay(1000); // hold on for boot
    id = read8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID) {
      return false;  // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  write8(BNO055_PAGE_ID_ADDR, 0);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */

  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(mode);
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
    @brief  Changes the chip's axis remap
*/
/**************************************************************************/
void Adafruit_BNO055::setAxisRemap( adafruit_bno055_axis_remap_config_t remapcode )
{
  adafruit_bno055_opmode_t modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}

/**************************************************************************/
/*!
    @brief  Changes the chip's axis signs
*/
/**************************************************************************/
void Adafruit_BNO055::setAxisSign( adafruit_bno055_axis_remap_sign_t remapsign )
{
  adafruit_bno055_opmode_t modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}


/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void Adafruit_BNO055::setExtCrystalUse(boolean usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write8(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}


/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void Adafruit_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

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

  if (system_error != 0)
    *system_error     = read8(BNO055_SYS_ERR_ADDR);

  delay(200);
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

  /* Check the accelerometer revision */
  info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);

  a = read8(BNO055_SW_REV_ID_LSB_ADDR);
  b = read8(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
void Adafruit_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
int8_t Adafruit_BNO055::getTemp(void)
{
  int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
  return temp;
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

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

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
      /* 1dps = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
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
  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
  const double scale = (1.0 / (1<<14));
  imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
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
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ORIENTATION;
  event->timestamp = millis();

  /* Get a Euler angle sample for orientation */
  imu::Vector<3> euler = getVector(Adafruit_BNO055::VECTOR_EULER);
  event->orientation.x = euler.x();
  event->orientation.y = euler.y();
  event->orientation.z = euler.z();

  return true;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into a byte array
*/
/**************************************************************************/
bool Adafruit_BNO055::getSensorOffsets(uint8_t* calibData)
{
    if (isFullyCalibrated())
    {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);

        readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

        setMode(lastMode);
        return true;
    }
    return false;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into an offset struct
*/
/**************************************************************************/
bool Adafruit_BNO055::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type)
{
    if (isFullyCalibrated())
    {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        delay(25);

        /* Accel offset range depends on the G-range:
           +/-2g  = +/- 2000 mg
           +/-4g  = +/- 4000 mg
           +/-8g  = +/- 8000 mg
           +/-1§g = +/- 16000 mg */
        offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Z_LSB_ADDR));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type.mag_offset_x = (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y = (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z = (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

        /* Gyro offset range depends on the DPS range:
          2000 dps = +/- 32000 LSB
          1000 dps = +/- 16000 LSB
           500 dps = +/- 8000 LSB
           250 dps = +/- 4000 LSB
           125 dps = +/- 2000 LSB
           ... where 1 DPS = 16 LSB */
        offsets_type.gyro_offset_x = (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type.accel_radius = (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type.mag_radius = (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

        setMode(lastMode);
        return true;
    }
    return false;
}


/**************************************************************************/
/*!
@brief  Writes an array of calibration values to the sensor's offset registers
*/
/**************************************************************************/
void Adafruit_BNO055::setSensorOffsets(const uint8_t* calibData)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    /* A writeLen() would make this much cleaner */
    write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    write8(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
    write8(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
    write8(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
    write8(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
    write8(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
    write8(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

    write8(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
    write8(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
    write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
    write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
    write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
    write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

    write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
    write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

    setMode(lastMode);
}

/**************************************************************************/
/*!
@brief  Writes to the sensor's offset registers from an offset struct
*/
/**************************************************************************/
void Adafruit_BNO055::setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    setMode(lastMode);
}

/**************************************************************************/
/*!
    @brief  Checks of all cal status values are set to 3 (fully calibrated)
*/
/**************************************************************************/
bool Adafruit_BNO055::isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return false;
    return true;
}

/**************************************************************************/
/*!
@brief  Enables interrupt type specified and disables/enables interrupt pin
*/
/**************************************************************************/
bool Adafruit_BNO055::enableInterrupts( adafruit_bno055_intr_en_t int_en_code, bool triggerPin)
{
  // create status variable
  int8_t status = 0; // if greater than zero a failure has occured

  // enter config mode
  adafruit_bno055_opmode_t modeback = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  // save selected page ID and switch to page 1
  uint8_t savePageID = read8(BNO055_PAGE_ID_ADDR);
  write8(BNO055_PAGE_ID_ADDR, 0x01);

  // enable specific interrupt type requested
  switch (int_en_code) //TODO: change these definitions (ACC_NM etc) into a number sequence
  {
    case ACC_NM:
      if(triggerPin){
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (1 << 7)); // enable pin change
      } else {
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (0 << 7)); // enable pin change
      }
      write8(BNO055_INTR_EN_ADDR, (read8(BNO055_INTR_EN_ADDR)) | (1 << 7)); // enable interrupt
      write8(BNO055_INTR_ACCEL_NM_SETT, (read8(BNO055_INTR_ACCEL_NM_SETT)) | (0 << 0)); // decides SM/NM
    break;
    case ACC_AM:
      if(triggerPin){
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (1 << 6)); // enable pin change
      } else {
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (0 << 6)); // enable pin change
      }
      write8(BNO055_INTR_EN_ADDR, (read8(BNO055_INTR_EN_ADDR)) | (1 << 6)); // enable interrupt
    break;
    case ACC_SM:
      if(triggerPin){
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (1 << 7)); // enable pin change
      } else {
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (0 << 7)); // enable pin change
      }
      write8(BNO055_INTR_EN_ADDR, (read8(BNO055_INTR_EN_ADDR)) | (1 << 7)); // enable interrupt
      write8(BNO055_INTR_ACCEL_NM_SETT, (read8(BNO055_INTR_ACCEL_NM_SETT)) | (1 << 0)); // decides SM/NM
    break;
    case ACC_HIGH_G:
      if(triggerPin){
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (1 << 5)); // enable pin change
      } else {
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (0 << 5)); // enable pin change
      }
      write8(BNO055_INTR_EN_ADDR, (read8(BNO055_INTR_EN_ADDR)) | (1 << 5)); // enable interrupt
    break;
    case GYR_HIGH_RATE:
      if(triggerPin){
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (1 << 3)); // enable pin change
      } else {
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (0 << 3)); // enable pin change
      }
      write8(BNO055_INTR_EN_ADDR, (read8(BNO055_INTR_EN_ADDR)) | (1 << 3)); // enable interrupt
    break;
    case GYRO_AM:
      if(triggerPin){
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (1 << 2)); // enable pin change
      } else {
        status = write8(BNO055_INTR_MSK_ADDR, (read8(BNO055_INTR_MSK_ADDR)) | (0 << 2)); // enable pin change
      }
      write8(BNO055_INTR_EN_ADDR, (read8(BNO055_INTR_EN_ADDR)) | (1 << 2)); // enable interrupt
    break;
  }
  // restore page ID
  write8(BNO055_PAGE_ID_ADDR, savePageID);

  // Set the requested operating mode (see section 3.3)
  setMode(modeback);
  delay(20);

  if (status > 0){
    return false;
  } else {
    return true;
  }

}

/**************************************************************************/
/*!
@brief  Sets interrupt axes
*/
/**************************************************************************/

bool Adafruit_BNO055::enableInterruptAxes( adafruit_bno055_intr_en_t int_en_code, String axes )
{
  // create status variable
  int8_t status = 0; // if greater than zero a failure has occured

  // enter config mode
  adafruit_bno055_opmode_t modeback = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  // save selected page ID and switch to page 1
  uint8_t savePageID = read8(BNO055_PAGE_ID_ADDR);
  write8(BNO055_PAGE_ID_ADDR, 0x01);

  // parse axis flags
  int X_EN = 0; int Y_EN = 0; int Z_EN = 0;
  // set flags based on input String
  if (axes.indexOf("x") != -1){
    X_EN = 1;
    Serial.println("X axis trigger enabled.");
  }
  if (axes.indexOf("y") != -1){
    Y_EN = 1;
    Serial.println("Y axis trigger enabled.");
  }
  if (axes.indexOf("z") != -1){
    Z_EN = 1;
    Serial.println("Z axis trigger enabled.");
  }

  switch(int_en_code){
    case ACC_NM:
      if(X_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 2));
      }
      if(Y_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 3));
      }
      if(Z_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 4));
      }
    break;
    case ACC_AM:
      if(X_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 2));
      }
      if(Y_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 3));
      }
      if(Z_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 4));
      }
    break;
    case ACC_SM:
      if(X_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 2));
      }
      if(Y_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 3));
      }
      if(Z_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 4));
      }
    break;
    case ACC_HIGH_G:
      if(X_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 5));
      }
      if(Y_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 6));
      }
      if(Z_EN){
        status = write8(BNO055_INTR_ACCEL_SETT, (read8(BNO055_INTR_ACCEL_SETT)) | (1 << 7));
      }
    break;
    case GYR_HIGH_RATE:
      if(X_EN){
        status = write8(BNO055_INTR_GYR_SETT, (read8(BNO055_INTR_GYR_SETT)) | (1 << 3));
      }
      if(Y_EN){
        status = write8(BNO055_INTR_GYR_SETT, (read8(BNO055_INTR_GYR_SETT)) | (1 << 4));
      }
      if(Z_EN){
        status = write8(BNO055_INTR_GYR_SETT, (read8(BNO055_INTR_GYR_SETT)) | (1 << 5));
      }
    break;
    case GYRO_AM:
      if(X_EN){
        status = write8(BNO055_INTR_GYR_SETT, (read8(BNO055_INTR_GYR_SETT)) | (1 << 2));
      }
      if(Y_EN){
        status = write8(BNO055_INTR_GYR_SETT, (read8(BNO055_INTR_GYR_SETT)) | (1 << 1));
      }
      if(Z_EN){
        status = write8(BNO055_INTR_GYR_SETT, (read8(BNO055_INTR_GYR_SETT)) | (1 << 0));
      }
    break;
  }
  // restore page ID
  write8(BNO055_PAGE_ID_ADDR, savePageID);

  // Set the requested operating mode (see section 3.3)
  setMode(modeback);
  delay(20);

  if (status > 0){
    return false;
  } else {
    return true;
  }

}
// TODO: setThreshold and setDuration, setHysteresis. Also disableInterrupt, disableInterruptAxes and checkInterruptStates

/**************************************************************************/
/*!
@brief  Retrieve interrupt states and settings - DEBUG FUNCTION
*/
/**************************************************************************/
void Adafruit_BNO055::checkInterruptStates( void )
{
  // enter config mode
  adafruit_bno055_opmode_t modeback = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  // check the interrupt status register
  uint8_t interrupt_status = readIntStatus();
  // save selected page ID and switch to page 1
  uint8_t savePageID = read8(BNO055_PAGE_ID_ADDR);
  write8(BNO055_PAGE_ID_ADDR, 0x01);
  // check all config registers
  int8_t accel_config = read8(BNO055_ACCEL_CONFIG);
  int8_t magneto_config = read8(BNO055_MAG_CONFIG);
  int8_t gyro_config0 = read8(BNO055_GYRO_CONFIG0);
  int8_t gyro_config1 = read8(BNO055_GYRO_CONFIG1);
  // check the interrupt enable registers
  int8_t interrupt_enables = read8(BNO055_INTR_EN_ADDR);
  // check the interrupt mask
  int8_t interrupt_mask = read8(BNO055_INTR_MSK_ADDR);
  // check the accel axis enable states
  int8_t accel_axis_enables = read8(BNO055_INTR_ACCEL_SETT);
  // check the gyro axis enable states
  int8_t gyro_axis_enables = read8(BNO055_INTR_GYR_SETT);
  // check the threshold levels
  int8_t accel_nm_thresh = read8(BNO055_INTR_ACCEL_NM_THRES);
  int8_t accel_am_thresh = read8(BNO055_INTR_ACCEL_AM_THRES);
  int8_t accel_hg_thresh = read8(BNO055_INTR_ACCEL_HG_THRES);
  int8_t gyro_am_thresh = read8(BNO055_INTR_GYR_AM_THRES);
  // check the duration levels
  int8_t accel_nm_dur = read8(BNO055_INTR_ACCEL_NM_SETT);
  int8_t accel_hg_dur = read8(BNO055_INTR_ACCEL_HG_DUR);
  int8_t gyro_x_dur = read8(BNO055_INTR_GYR_DUR_X);
  int8_t gyro_y_dur = read8(BNO055_INTR_GYR_DUR_Y);
  int8_t gyro_z_dur = read8(BNO055_INTR_GYR_DUR_Z);
  int8_t gyro_am_set = read8(BNO055_INTR_GYR_AM_SET);
  // check the hysteresis settings
  int8_t gyro_x_hyst = read8(BNO055_INTR_GYR_HR_X_SET);
  int8_t gyro_y_hyst = read8(BNO055_INTR_GYR_HR_Y_SET);
  int8_t gyro_z_hyst = read8(BNO055_INTR_GYR_HR_Z_SET);
  // restore page ID
  write8(BNO055_PAGE_ID_ADDR, savePageID);
  // Set the previous operating mode (see section 3.3)
  setMode(modeback);
  delay(20);
  // print current call on interrupt status (clears status register)
  Serial.print("     INTERRUPT_STA: "); printWithZeros(interrupt_status, 'e');
  // print all config registers
  Serial.print("      ACCEL_CONFIG: "); printWithZeros(accel_config, 'e');
  Serial.print("    MAGNETO_CONFIG: "); printWithZeros(magneto_config, 'e');
  Serial.print("      GYRO_CONFIG0: "); printWithZeros(gyro_config0, 'e');
  Serial.print("      GYRO_CONFIG1: "); printWithZeros(gyro_config1, 'e');
  // print the interrupt enable registers
  Serial.print(" INTERRUPT_ENABLES: "); printWithZeros(interrupt_enables, 'e');
  // print the interrupt mask
  Serial.print("    INTERRUPT_MASK: "); printWithZeros(interrupt_mask, 'e');
  // print the accel axis enable states
  Serial.print("ACCEL_AXIS_ENABLES: "); printWithZeros(accel_axis_enables, 'e');
  // print the gyro axis enable states
  Serial.print(" GYRO_AXIS_ENABLES: "); printWithZeros(gyro_axis_enables, 'e');
  // print the threshold levels
  Serial.print("   ACCEL_NM_THRESH: "); printWithZeros(accel_nm_thresh, 'e');
  Serial.print("   ACCEL_AM_THRESH: "); printWithZeros(accel_am_thresh, 'e');
  Serial.print("   ACCEL_HG_THRESH: "); printWithZeros(accel_hg_thresh, 'e');
  Serial.print("    GYRO_AM_THRESH: "); printWithZeros(gyro_am_thresh, 'e');
  // print the duration levels
  Serial.print("      ACCEL_NM_DUR: "); printWithZeros(accel_nm_dur, 'e');
  Serial.print("      ACCEL_HG_DUR: "); printWithZeros(accel_hg_dur, 'e');
  Serial.print("        GYRO_X_DUR: "); printWithZeros(gyro_x_dur, 'e');
  Serial.print("        GYRO_Y_DUR: "); printWithZeros(gyro_y_dur, 'e');
  Serial.print("        GYRO_Z_DUR: "); printWithZeros(gyro_z_dur, 'e');
  Serial.print("       GYRO_AM_SET: "); printWithZeros(gyro_am_set, 'e');
  // print the hysteresis settings
  Serial.print("       GYRO_X_HYST: "); printWithZeros(gyro_x_hyst, 'e');
  Serial.print("       GYRO_Y_HYST: "); printWithZeros(gyro_y_hyst, 'e');
  Serial.print("       GYRO_Z_HYST: "); printWithZeros(gyro_z_hyst, 'e');
}

/**************************************************************************/
/*!
@brief  setIntThreshold
*/
/**************************************************************************/
bool Adafruit_BNO055::setIntThreshold( adafruit_bno055_intr_en_t int_en_code, int duration )
{
  // TODO: create different code set to group similar processes and increase code reuse
  
  // create status variable
  int8_t status = 0; // if greater than zero a failure has occured

  // enter config mode
  adafruit_bno055_opmode_t modeback = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  // save selected page ID and switch to page 1
  uint8_t savePageID = read8(BNO055_PAGE_ID_ADDR);
  write8(BNO055_PAGE_ID_ADDR, 0x01);

  switch(int_en_code){
    case ACC_NM:
      // check duration range is 8-bit
      if(duration > 0 && duration < 256){
        // mask register value to just the first two bits
        uint8_t masked_config = 0b00000011 & read8(BNO055_ACCEL_CONFIG);
        // write to the register
        status = write8(BNO055_INTR_ACCEL_NM_THRES, duration);
        // display real value
        float thresh_mg = 0.0;
        if (masked_config == 0){
          // threshold is in 2g range, 3.91mg per bit
          thresh_mg = duration * 3.91;
        } else if (masked_config == 1){
          // threshold is in 4g range, 7.81mg per bit
          thresh_mg = duration * 7.81;
        } else if (masked_config == 2){
          // threshold is in 8g range, 15.63mg per bit
          thresh_mg = duration * 15.63;
        } else if (masked_config == 3){
          // threshold is in 16g range, 31.25mg per bit
          thresh_mg = duration * 31.25;
        } else {
          Serial.println("Threshhold not set!");
        }
        delay(500);
        Serial.print("NM threshold set to: "); Serial.print(thresh_mg);
        Serial.println("mg.");
      } else {
        // display error
        Serial.println("Duration value not within 1 - 255 range!");
      }
    break;
    case ACC_AM:
    // check duration range is 8-bit
    if(duration > 0 && duration < 256){
      // mask register value to just the first two bits
      uint8_t masked_config = 0b00000011 & read8(BNO055_ACCEL_CONFIG);
      // write to the register
      status = write8(BNO055_INTR_ACCEL_AM_THRES, duration);
      // display real value
      float thresh_mg = 0.0;
      if (masked_config == 0){
        // threshold is in 2g range, 3.91mg per bit
        thresh_mg = duration * 3.91;
      } else if (masked_config == 1){
        // threshold is in 4g range, 7.81mg per bit
        thresh_mg = duration * 7.81;
      } else if (masked_config == 2){
        // threshold is in 8g range, 15.63mg per bit
        thresh_mg = duration * 15.63;
      } else if (masked_config == 3){
        // threshold is in 16g range, 31.25mg per bit
        thresh_mg = duration * 31.25;
      } else {
        Serial.println("Threshhold not set!");
      }
      delay(500);
      Serial.print("AM threshold set to: "); Serial.print(thresh_mg);
      Serial.println("mg.");
    } else {
      // display error
      Serial.println("Duration value not within 1 - 255 range!");
    }
    break;
    case ACC_SM:
      if(duration > 0 && duration < 256){
        // write to the register
        status = write8(BNO055_INTR_ACCEL_NM_THRES, duration);
      } else {
        // display error
        Serial.println("Duration value not within 1 - 255 range!");
      }
    break;
    case ACC_HIGH_G:
      if(duration > 0 && duration < 256){
        // write to the register
        status = write8(BNO055_INTR_ACCEL_NM_THRES, duration);
      } else {
        // display error
        Serial.println("Duration value not within 1 - 255 range!");
      }
    break;
    case GYR_HIGH_RATE:

    break;
    case GYRO_AM:

    break;
  }

  // restore page ID
  write8(BNO055_PAGE_ID_ADDR, savePageID);
  // Set the previous operating mode (see section 3.3)
  setMode(modeback);
  delay(20);

  if (status > 0){
    return false;
  } else {
    return true;
  }
}

/**************************************************************************/
/*!
@brief  On interrupt reads the interrupt status register
*/
/**************************************************************************/
int8_t Adafruit_BNO055::readIntStatus()
{
  // enter config mode
  adafruit_bno055_opmode_t modeback = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  // save selected page ID and switch to page 0
  uint8_t savePageID = read8(BNO055_PAGE_ID_ADDR);
  write8(BNO055_PAGE_ID_ADDR, 0x00);

  // read from INT_STA to determine the type of interrupt occurance
  int8_t intstatus = read8(BNO055_INTR_STAT_ADDR);

  /* Interrupt Status (see section 4.3.56)
     ---------------------------------
     0 = No Status
     4 = Gyroscope any-motion interrupt
     8 = Gyroscope high-rate interrupt
     32 = Accelerometer any-motion interrupt
     49 = Accelerometer no-motion/slow-motion interrupt */

   // restore page ID
   write8(BNO055_PAGE_ID_ADDR, savePageID);

   // Set the requested operating mode (see section 3.3)
   setMode(modeback);
   delay(20);

  return intstatus;
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
int8_t Adafruit_BNO055::write8(adafruit_bno055_reg_t reg, byte value)
{
  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  int8_t status = Wire.endTransmission();

  return status;
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

/**************************************************************************/
/*!
    @brief  Prints 8-bit register value with leading zeros
*/
/**************************************************************************/
void Adafruit_BNO055::printWithZeros ( uint8_t input, char flag )
{
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
      if (mask & input) {
          Serial.print('1');
      }
      else {
          Serial.print('0');
      }
  }
  if (flag == 'e'){
    Serial.print('\n');
  }
}
