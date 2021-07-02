#include "AD7190.h"     // AD7190 definitions.

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes to be written.
 * @param modifyCS - Allows Chip Select to be modified.
 *
 * @return none.
*******************************************************************************/
void AD7190_SetRegisterValue(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS) {
  uint8_t writeCommand[5] = {0, 0, 0, 0, 0};
  uint8_t* dataPointer = (uint8_t*)&registerValue;
  uint8_t bytesNr = bytesNumber;

  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);
  while(bytesNr > 0) {
    writeCommand[bytesNr] = *dataPointer;
    dataPointer ++;
    bytesNr --;
  }
  if(modifyCS) AD7190_SpiDriver->AD7190_CS_Low();
  AD7190_SpiDriver->AD7190_Spi_Write(writeCommand, bytesNumber + 1);
  if(modifyCS) AD7190_SpiDriver->AD7190_CS_High();
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 * @param modifyCS    - Allows Chip Select to be modified.
 *
 * @return buffer - Value of the register.
*******************************************************************************/
uint32_t AD7190_GetRegisterValue(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS) {
  uint8_t registerWord[5] = {0, 0, 0, 0, 0}; 
  uint32_t buffer = 0x0;
  uint8_t i = 0;

  registerWord[0] = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);
  if(modifyCS) AD7190_SpiDriver->AD7190_CS_Low();
  AD7190_SpiDriver->AD7190_Spi_Read(registerWord, bytesNumber + 1);
  if(modifyCS) AD7190_SpiDriver->AD7190_CS_High();
  for(i = 1; i < bytesNumber + 1; i++) {
      buffer = (buffer << 8) + registerWord[i];
  }

  return buffer;
}

/***************************************************************************//**
 * @brief Checks if the AD7190 part is present.
 *
 * @return status - Indicates if the part is present or not.
*******************************************************************************/
uint8_t AD7190_Init(AD7190_SpiDriver_Typedef* AD7190_SpiDriver) {
  uint8_t status = 1;
  uint8_t regVal = 0;

  AD7190_Reset(AD7190_SpiDriver);
  /* Allow at least 500 us before accessing any of the on-chip registers. */
  AD7190_SpiDriver->AD7190_Delay_ms(5);
  regVal = AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_ID, 1, 1);
  if( (regVal & AD7190_ID_MASK) != ID_AD7190) {
      status = 0;
  }
  return status ;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/
void AD7190_Reset(AD7190_SpiDriver_Typedef* AD7190_SpiDriver) {
  uint8_t registerWord[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  AD7190_SpiDriver->AD7190_Spi_Write(registerWord, 7);
}

/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/
void AD7190_SetPower(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t pwrMode) {
  uint32_t oldPwrMode = 0x0;
  uint32_t newPwrMode = 0x0;
  oldPwrMode = AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_MODE, 3, 1);
  oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
  newPwrMode = oldPwrMode | AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) | (!pwrMode * (AD7190_MODE_PWRDN)));
  AD7190_SetRegisterValue(AD7190_SpiDriver, AD7190_REG_MODE, newPwrMode, 3, 1);
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void AD7190_WaitRdyGoLow(AD7190_SpiDriver_Typedef* AD7190_SpiDriver) {
  uint32_t timeOutCnt = 0xFFFFF;
  while(AD7190_SpiDriver->AD7190_CheckDataReadyPin() && timeOutCnt--) { }
}

/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *  
 * @return none.
*******************************************************************************/
void AD7190_ChannelSelect(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint16_t channel) {
  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;   

  oldRegValue = AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_CONF, 3, 1);
  oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
  newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);   
  AD7190_SetRegisterValue(AD7190_SpiDriver, AD7190_REG_CONF, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7190_Calibrate(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t mode, uint8_t channel) {
  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;

  AD7190_ChannelSelect(AD7190_SpiDriver, channel);
  oldRegValue = AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_MODE, 3, 1);
  oldRegValue &= ~AD7190_MODE_SEL(0x7);
  newRegValue = oldRegValue | AD7190_MODE_SEL(mode);

  AD7190_SpiDriver->AD7190_CS_Low();
  AD7190_SetRegisterValue(AD7190_SpiDriver, AD7190_REG_MODE, newRegValue, 3, 0); // CS is not modified.
  AD7190_WaitRdyGoLow(AD7190_SpiDriver);
  AD7190_SpiDriver->AD7190_CS_High();
}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range - Gain select bits. These bits are written by the user to select 
                 the ADC input range.     
 *
 * @return none.
*******************************************************************************/
void AD7190_RangeSetup(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t polarity, uint8_t range) {
  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;

  oldRegValue = AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_CONF,3, 1);  
  oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(0x7));
  newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range);
  AD7190_SetRegisterValue(AD7190_SpiDriver, AD7190_REG_CONF, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
uint32_t AD7190_SingleConversion(AD7190_SpiDriver_Typedef* AD7190_SpiDriver) {
  uint32_t command = 0x0;
  uint32_t regData = 0x0;

  command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);
  AD7190_SpiDriver->AD7190_CS_Low();
  AD7190_SetRegisterValue(AD7190_SpiDriver, AD7190_REG_MODE, command, 3, 0); // CS is not modified.
  AD7190_WaitRdyGoLow(AD7190_SpiDriver);
  regData = AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_DATA, 3, 0);
  AD7190_SpiDriver->AD7190_CS_High();
  
  return regData;
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
uint32_t AD7190_ContinuousReadAvg(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t sampleNumber) {
  uint32_t samplesAverage = 0x0;
  uint8_t count = 0x0;
  uint32_t command = 0x0;

  command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);
  AD7190_SpiDriver->AD7190_CS_Low();
  AD7190_SetRegisterValue(AD7190_SpiDriver, AD7190_REG_MODE, command, 3, 0); // CS is not modified.
  for(count = 0;count < sampleNumber;count ++) {
    AD7190_WaitRdyGoLow(AD7190_SpiDriver);
    samplesAverage += AD7190_GetRegisterValue(AD7190_SpiDriver, AD7190_REG_DATA, 3, 0); // CS is not modified.
  }
  AD7190_SpiDriver->AD7190_CS_High();
  samplesAverage = samplesAverage / sampleNumber;

  return samplesAverage;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
uint32_t AD7190_TemperatureRead(AD7190_SpiDriver_Typedef* AD7190_SpiDriver) {
  uint8_t temperature = 0x0;
  uint32_t dataReg = 0x0;

  AD7190_RangeSetup(AD7190_SpiDriver, 0, AD7190_CONF_GAIN_1);
  AD7190_ChannelSelect(AD7190_SpiDriver, AD7190_CH_TEMP_SENSOR);
  dataReg = AD7190_SingleConversion(AD7190_SpiDriver);
  dataReg -= 0x800000;
  dataReg /= 2815;   // Kelvin Temperature
  dataReg -= 273;    //Celsius Temperature
  temperature = (uint32_t) dataReg;

  return temperature;
}
