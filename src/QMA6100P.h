/**
 * @brief The following class implements the methods to set, get, and read from the Triple
 * Axis Accelerometer - QMA6100P.
 */

#pragma once

#include <Wire.h>
#include "QMA6100P_regs.h"

#define QMA6100P_ADDRESS_HIGH 0x13
#define QMA6100P_ADDRESS_LOW 0x12

#define QMA6100P_CHIP_ID 0x90

/**
 * @brief CNTL1 GSEL<1:0>
 */
#define SFE_QMA6100P_RANGE2G 0b0001
#define SFE_QMA6100P_RANGE4G 0b0010
#define SFE_QMA6100P_RANGE8G 0b0100
#define SFE_QMA6100P_RANGE16G 0b1000
#define SFE_QMA6100P_RANGE32G 0b1111

#define SFE_QMA6100P_FIFO_MODE_BYPASS 0b00
#define SFE_QMA6100P_FIFO_MODE_FIFO   0b01
#define SFE_QMA6100P_FIFO_MODE_STREAM 0b10
#define SFE_QMA6100P_FIFO_MODE_FIFO   0b11

#define SENSORS_GRAVITY_EARTH (9.80665F)

/**
 * @brief Structure to hold the output data.
 */
struct outputData
{
  float xData;
  float yData;
  float zData;
};

/**
 * @brief Structure to hold the raw output data.
 */
struct rawOutputData
{
  int16_t xData;
  int16_t yData;
  int16_t zData;
};

/**
 * @brief Class to interact with the QMA6100P Triple Axis Accelerometer.
 */
class QMA6100P
{
public:

  /**
   * @brief Initialize the QMA6100P sensor.
   * 
   * @return bool True if initialization is successful, false otherwise.
   */
  bool begin();

  /**
   * @brief Calibrate the offsets of the sensor.
   * 
   * @return bool True if calibration is successful, false otherwise.
   */
  bool calibrateOffsets();

  /**
   * @brief Get the unique ID of the QMA6100P sensor.
   * 
   * @return uint8_t The unique ID of the sensor. Returns 0xFF if the read operation fails.
   */
  uint8_t getUniqueID();

  /**
   * @brief Write a byte to a specific register.
   * 
   * @param registerAddress The address of the register to write to.
   * @param data The data to write to the register.
   * @return bool True if the write operation is successful, false otherwise.
   */
  bool writeRegisterByte(uint8_t registerAddress, uint8_t data);

  /**
   * @brief Read a region of registers.
   * 
   * @param registerAddress The starting address of the registers to read.
   * @param sensorData Pointer to the buffer to store the read data.
   * @param len The number of bytes to read.
   * @return bool True if the read operation is successful, false otherwise.
   */
  bool readRegisterRegion(uint8_t registerAddress, uint8_t* sensorData, int len);
  
  /**
   * @brief Get the accelerometer data.
   * 
   * @param userData Pointer to the structure to store the accelerometer data.
   * @return bool True if the operation is successful, false otherwise.
   */
  bool getAccelData(outputData *userData);

  /**
   * @brief Convert raw accelerometer data to user-friendly format.
   * 
   * @param userAccel Pointer to the structure to store the converted accelerometer data.
   * @param rawAccelData Pointer to the structure holding the raw accelerometer data.
   * @return bool True if the conversion is successful, false otherwise.
   */
  bool convAccelData(outputData *userAccel, rawOutputData *rawAccelData);

  /**
   * @brief Enable or disable the accelerometer.
   * 
   * @param enable True to enable the accelerometer, false to disable it.
   * @return bool True if the operation is successful, false otherwise.
   */
  bool enableAccel(bool enable = true);

  /**
   * @brief Perform a software reset on the QMA6100P sensor.
   * 
   * @return bool True if the reset operation is successful, false otherwise.
   */
  bool softwareReset();

  /**
   * @brief Get the current operating mode of the QMA6100P sensor.
   * 
   * @return uint8_t The current operating mode bit. Returns false if the read operation fails.
   */
  uint8_t getOperatingMode();

  /**
   * @brief Set the operational g-range of the accelerometer.
   * 
   * @param range The g-range to set.
   * @return bool True if the operation is successful, false otherwise.
   */
  bool setRange(uint8_t range);

  /**
   * @brief Enable or disable the data engine.
   * 
   * @param enable True to enable the data engine, false to disable it.
   * @return bool True if the operation is successful, false otherwise.
   */
  bool enableDataEngine(bool enable = true);

  /**
   * @brief Get the raw accelerometer register data.
   * 
   * @param rawAccelData Pointer to the structure to store the raw accelerometer data.
   * @return bool True if the operation is successful, false otherwise.
   */
  bool getRawAccelRegisterData(rawOutputData *rawAccelData);

  /**
   * @brief Apply offset values to the accelerometer data.
   * 
   * @param x Reference to the x-axis data.
   * @param y Reference to the y-axis data.
   * @param z Reference to the z-axis data.
   */
  void offsetValues(float &x, float &y, float &z);

  /**
   * @brief Set the offset values for the accelerometer.
   * 
   * @param x The offset value for the x-axis.
   * @param y The offset value for the y-axis.
   * @param z The offset value for the z-axis.
   */
  void setOffset(float x, float y, float z);

  /**
   * @brief Set the FIFO mode of the QMA6100P sensor.
   * 
   * @param fifo_mode The FIFO mode to set.
   * @return bool True if the operation is successful, false otherwise.
   */
  bool setFifoMode(uint8_t fifo_mode);

  /**
   * @brief Get the current g-range of the accelerometer.
   * 
   * @return uint8_t The current g-range.
   */
  uint8_t getRange();

  /**
   * @brief Structure to hold the raw accelerometer data.
   */
  rawOutputData rawAccelData;

  /**
   * @brief QMA6100P conversion values for different g-ranges.
   */
  const double convRange2G = .000244;
  const double convRange4G = .000488;
  const double convRange8G = .000977;
  const double convRange16G = .001950;
  const double convRange32G = .003910;

  /**
   * @brief Offset values for the accelerometer.
   */
  float xOffset = 0.0;
  float yOffset = 0.0;
  float zOffset = 0.0;

protected:
  /**
   * @brief Keep a local copy of the range. Default to "unknown" (-1).
   */
  int _range = -1;
};