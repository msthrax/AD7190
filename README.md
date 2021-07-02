# AD7190
 AD7190 general C driver library.

/*! Writes data into a register. */
void AD7190_SetRegisterValue(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS);

/*! Reads the value of a register. */
uint32_t AD7190_GetRegisterValue(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS);

/*! Checks if the AD7139 part is present. */
uint8_t AD7190_Init(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);

/*! Resets the device. */
void AD7190_Reset(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);

/*! Set device to idle or power-down. */
void AD7190_SetPower(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t pwrMode);

/*! Waits for RDY pin to go low. */
void AD7190_WaitRdyGoLow(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);

/*! Selects the channel to be enabled. */
void AD7190_ChannelSelect(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint16_t channel);

/*! Performs the given calibration to the specified channel. */
void AD7190_Calibrate(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t mode, uint8_t channel);

/*! Selects the polarity of the conversion and the ADC input range. */
void AD7190_RangeSetup(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t polarity, uint8_t range);

/*! Returns the result of a single conversion. */
uint32_t AD7190_SingleConversion(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);

/*! Returns the average of several conversion results. */
uint32_t AD7190_ContinuousReadAvg(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t sampleNumber);

/*! Read data from temperature sensor and converts it to Celsius degrees. */
uint32_t AD7190_TemperatureRead(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);

"AD7190_TemperatureRead()" will change range. Be carefull.