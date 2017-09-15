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

  setPage(PAGE_0);

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
 @brief  Puts the chip on the specified register page
 */
/**************************************************************************/
void Adafruit_BNO055::setPage(adafruit_bno055_page_t page)
{
    _page = page;
    write8(BNO055_PAGE_ID_ADDR, _page);
    delay(30);
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
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void Adafruit_BNO055::setExtCrystalUse(boolean usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  setPage(PAGE_0);
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
    @brief  Resets all interrupts
 */
/**************************************************************************/
void Adafruit_BNO055::resetInterrupts()
{
    // Read to get the current settings
    int8_t sysTrigger = (int8_t)(read8(BNO055_SYS_TRIGGER_ADDR));
    
    // Set the flags as requested
    sysTrigger = sliceValueIntoRegister(0x01, sysTrigger, BNO055_SYS_TRIGGER_ADDR_RST_INT_MSK, BNO055_SYS_TRIGGER_ADDR_RST_INT_POS);
    
    // Write back the entire register
    write8(BNO055_SYS_TRIGGER_ADDR, sysTrigger);
    delay(30);
}

/**************************************************************************/
/*!
 @brief  Enables the interrupts on specific axes
 */
/**************************************************************************/
void Adafruit_BNO055::enableInterruptsOnXYZ(uint8_t x, uint8_t y, uint8_t z)
{
    // TODO: this may be a good flow for any config register setting methods,
    // so could be worth a look at the those methods for rework
    
    // Need to be on page 0 to get into config mode
    adafruit_bno055_page_t lastPage = _page;
    if (lastPage != PAGE_0) setPage(PAGE_0);

    // Must be in config mode, so force it
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);

    // Change to page 1 for interrupt settings
    setPage(PAGE_1);
    
    // Read to get the current settings
    int8_t intSettings = (int8_t)(read8(ACC_INT_Settings_ADDR));
    
    // Set the flags as requested--binary choice, so set or unset
    intSettings = sliceValueIntoRegister(x ? 0x01 : 0x00, intSettings, ACC_INT_Settings_ACC_X_MSK, ACC_INT_Settings_ACC_X_POS);
    intSettings = sliceValueIntoRegister(y ? 0x01 : 0x00, intSettings, ACC_INT_Settings_ACC_Y_MSK, ACC_INT_Settings_ACC_Y_POS);
    intSettings = sliceValueIntoRegister(z ? 0x01 : 0x00, intSettings, ACC_INT_Settings_ACC_Z_MSK, ACC_INT_Settings_ACC_Z_POS);

    // Write back the entire register
    write8(ACC_INT_Settings_ADDR, intSettings);
    delay(30);
    
    // Return the mode to the last mode
    setPage(PAGE_0);
    setMode(lastMode);
    
    // Change the page back to whichever it was initially
    if (lastPage != PAGE_0) setPage(lastPage);
}

/**************************************************************************/
/*!
 @brief  Enables the the slow/no motion interrupt, specifying settings
 */
/**************************************************************************/
void Adafruit_BNO055::enableSlowNoMotion(uint8_t threshold, uint8_t duration, uint8_t motionType)
{
    // TODO: this may be a good flow for any config register setting methods,
    // so could be worth a look at the those methods for rework
    
    // Need to be on page 0 to get into config mode
    adafruit_bno055_page_t lastPage = _page;
    if (lastPage != PAGE_0) setPage(PAGE_0);
    
    // Must be in config mode, so force it
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    
    // Change to page 1 for interrupt settings
    setPage(PAGE_1);

    // Enable which one: slow motion or no motion
    // Set duration (bits 1-6), first
    int8_t smnmSettings = (int8_t)(read8(ACC_NM_SET_ADDR));
    smnmSettings = sliceValueIntoRegister(motionType, smnmSettings, ACC_NM_SET_SLOWNO_MSK, ACC_NM_SET_SLOWNO_POS);
    smnmSettings = sliceValueIntoRegister(duration, smnmSettings, ACC_NM_SET_DUR_MSK, ACC_NM_SET_DUR_POS);
    write8(ACC_NM_SET_ADDR, smnmSettings);

    // Set the threshold (full byte)
    int8_t threshSettings = (int8_t)(read8(ACC_NM_THRES_ADDR));
    threshSettings = sliceValueIntoRegister(threshold, threshSettings, ACC_NM_THRES_MSK, ACC_NM_THRES_POS);
    write8(ACC_NM_THRES_ADDR, threshSettings);

    // Enable the interrupt
    setInterruptEnableAccelNM(ENABLE);
    
    // Fire on the pin
    setInterruptMaskAccelNM(ENABLE);
    
    delay(30);
    
    // Return the mode to the last mode
    setPage(PAGE_0);
    setMode(lastMode);
    
    // Change the page back to whichever it was initially
    if (lastPage != PAGE_0) setPage(lastPage);
}

/**************************************************************************/
/*!
 @brief  Disable the the slow/no motion interrupt
 */
/**************************************************************************/
void Adafruit_BNO055::disableSlowNoMotion()
{
    // TODO: this may be a good flow for any config register setting methods,
    // so could be worth a look at the those methods for rework
    
    // Need to be on page 0 to get into config mode
    adafruit_bno055_page_t lastPage = _page;
    if (lastPage != PAGE_0) setPage(PAGE_0);
    
    // Must be in config mode, so force it
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    
    // Change to page 1 for interrupt settings
    setPage(PAGE_1);
    
    // Disable the interrupt
    setInterruptEnableAccelNM(DISABLE);
    
    // Stop firing on the pin
    setInterruptMaskAccelNM(DISABLE);
    
    delay(30);
    
    // Return the mode to the last mode
    setPage(PAGE_0);
    setMode(lastMode);
    
    // Change the page back to whichever it was initially
    if (lastPage != PAGE_0) setPage(lastPage);
}

/**************************************************************************/
/*!
 @brief  Enables the the any motion interrupt, specifying settings
 */
/**************************************************************************/
void Adafruit_BNO055::enableAnyMotion(uint8_t threshold, uint8_t duration)
{
    // TODO: this may be a good flow for any config register setting methods,
    // so could be worth a look at the those methods for rework
    
    // Need to be on page 0 to get into config mode
    adafruit_bno055_page_t lastPage = _page;
    if (lastPage != PAGE_0) setPage(PAGE_0);
    
    // Must be in config mode, so force it
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    
    // Change to page 1 for interrupt settings
    setPage(PAGE_1);
    
    // Set duration (bits 1-6)
    int8_t intSettings = (int8_t)(read8(ACC_INT_Settings_ADDR));
    intSettings = sliceValueIntoRegister(duration, intSettings, ACC_INT_Settings_AM_DUR_MSK, ACC_INT_Settings_AM_DUR_POS);
    write8(ACC_INT_Settings_ADDR, intSettings);
    
    // Set the threshold (full byte)
    int8_t threshSettings = (int8_t)(read8(ACC_AM_THRES_ADDR));
    threshSettings = sliceValueIntoRegister(threshold, threshSettings, ACC_AM_THRES_MSK, ACC_AM_THRES_POS);
    write8(ACC_AM_THRES_ADDR, threshSettings);
    
    // Enable the interrupt
    setInterruptEnableAccelAM(ENABLE);
    
    // Fire on the pin
    setInterruptMaskAccelAM(ENABLE);
    
    delay(30);
    
    // Return the mode to the last mode
    setPage(PAGE_0);
    setMode(lastMode);
    
    // Change the page back to whichever it was initially
    if (lastPage != PAGE_0) setPage(lastPage);
}

/**************************************************************************/
/*!
 @brief  Disable the the any motion interrupt
 */
/**************************************************************************/
void Adafruit_BNO055::disableAnyMotion()
{
    // TODO: this may be a good flow for any config register setting methods,
    // so could be worth a look at the those methods for rework
    
    // Need to be on page 0 to get into config mode
    adafruit_bno055_page_t lastPage = _page;
    if (lastPage != PAGE_0) setPage(PAGE_0);
    
    // Must be in config mode, so force it
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    
    // Change to page 1 for interrupt settings
    setPage(PAGE_1);
    
    // Disable the interrupt
    setInterruptEnableAccelAM(DISABLE);
    
    // Stop firing on the pin
    setInterruptMaskAccelAM(DISABLE);
    
    delay(30);
    
    // Return the mode to the last mode
    setPage(PAGE_0);
    setMode(lastMode);
    
    // Change the page back to whichever it was initially
    if (lastPage != PAGE_0) setPage(lastPage);
}

/**************************************************************************/
/*!
 @brief  Enable or disable the slow/no motion interrupt entirely
 */
/**************************************************************************/
// WARNING: ONLY CALL INSIDE PAGE/CONFIG WRAPPER
void Adafruit_BNO055::setInterruptEnableAccelNM(uint8_t enable) {
    int8_t intEnable = (int8_t)(read8(INT_EN_ADDR));
    intEnable = sliceValueIntoRegister(enable, intEnable, INT_EN_ACC_NM_MSK, INT_EN_ACC_NM_POS);
    write8(INT_EN_ADDR, intEnable);
}

/**************************************************************************/
/*!
 @brief  Tell slow/no interrupt to set pin and output or just set the output
 */
/**************************************************************************/
// WARNING: ONLY CALL INSIDE PAGE/CONFIG WRAPPER
void Adafruit_BNO055::setInterruptMaskAccelNM(uint8_t enable) {
    int8_t intMask = (int8_t)(read8(INT_MSK_ADDR));
    intMask = sliceValueIntoRegister(enable, intMask, INT_MSK_ACC_NM_MSK, INT_MSK_ACC_NM_POS);
    write8(INT_MSK_ADDR, intMask);
}

/**************************************************************************/
/*!
 @brief  Enable or disable the any motion interrupt entirely
 */
/**************************************************************************/
// WARNING: ONLY CALL INSIDE PAGE/CONFIG WRAPPER
void Adafruit_BNO055::setInterruptEnableAccelAM(uint8_t enable) {
    int8_t intEnable = (int8_t)(read8(INT_EN_ADDR));
    intEnable = sliceValueIntoRegister(enable, intEnable, INT_EN_ACC_AM_MSK, INT_EN_ACC_AM_POS);
    write8(INT_EN_ADDR, intEnable);
}

/**************************************************************************/
/*!
 @brief  Tell any motion interrupt to set pin and output or just set the output
 */
/**************************************************************************/
// WARNING: ONLY CALL INSIDE PAGE/CONFIG WRAPPER
void Adafruit_BNO055::setInterruptMaskAccelAM(uint8_t enable) {
    int8_t intMask = (int8_t)(read8(INT_MSK_ADDR));
    intMask = sliceValueIntoRegister(enable, intMask, INT_MSK_ACC_AM_MSK, INT_MSK_ACC_AM_POS);
    write8(INT_MSK_ADDR, intMask);
}

/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void Adafruit_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  setPage(PAGE_0);

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

        offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Z_LSB_ADDR));

        offsets_type.gyro_offset_x = (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

        offsets_type.mag_offset_x = (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y = (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z = (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

        offsets_type.accel_radius = (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));
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

    /* A writeLen() would make this much cleaner */
    write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
    write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
    write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
    write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
    write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
    write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

    write8(MAG_OFFSET_X_LSB_ADDR, calibData[12]);
    write8(MAG_OFFSET_X_MSB_ADDR, calibData[13]);
    write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
    write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
    write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
    write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

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

    write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    setMode(lastMode);
}

bool Adafruit_BNO055::isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return false;
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
  delayMicroseconds(150);
    
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
 @brief  Sets the value of the partial (or full) register
 */
/**************************************************************************/
uint8_t Adafruit_BNO055::sliceValueIntoRegister(uint8_t value, uint8_t reg, uint8_t mask, uint8_t position) {
    return (reg & ~mask) | ((value << position) & mask);
}

