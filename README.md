# AD7190
 AD7190 general C driver library.

# List of functions
```c
void AD7190_SetRegisterValue(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t registerAddress, uint32_t registerValue, uint8_t bytesNumber, uint8_t modifyCS);
uint32_t AD7190_GetRegisterValue(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t registerAddress, uint8_t bytesNumber, uint8_t modifyCS);
uint8_t AD7190_Init(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);
void AD7190_Reset(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);
void AD7190_SetPower(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t pwrMode);
void AD7190_WaitRdyGoLow(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);
void AD7190_ChannelSelect(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint16_t channel);
void AD7190_Calibrate(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t mode, uint8_t channel);
void AD7190_RangeSetup(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t polarity, uint8_t range);
uint32_t AD7190_SingleConversion(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);
uint32_t AD7190_ContinuousReadAvg(AD7190_SpiDriver_Typedef* AD7190_SpiDriver, uint8_t sampleNumber);
uint32_t AD7190_TemperatureRead(AD7190_SpiDriver_Typedef* AD7190_SpiDriver);
```
# Warning

```c
AD7190_TemperatureRead()
```
will change range. Be carefull.
