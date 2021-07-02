#ifndef __AD7190_H__
#define __AD7190_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/******************************** AD7190 **************************************/
/******************************************************************************/

/* SPI slave device ID */
#define AD7190_SLAVE_ID         1

/* AD7190 Register Map */
#define AD7190_REG_COMM         0 // Communications Register (WO, 8-bit) 
#define AD7190_REG_STAT         0 // Status Register         (RO, 8-bit) 
#define AD7190_REG_MODE         1 // Mode Register           (RW, 24-bit 
#define AD7190_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD7190_REG_DATA         3 // Data Register           (RO, 24/32-bit) 
#define AD7190_REG_ID           4 // ID Register             (RO, 8-bit) 
#define AD7190_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit) 
#define AD7190_REG_OFFSET       6 // Offset Register         (RW, 24-bit 
#define AD7190_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

/* Communications Register Bit Designations (AD7190_REG_COMM) */
#define AD7190_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD7190_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7190_COMM_READ        (1 << 6)           // Read Operation. 
#define AD7190_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD7190_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

/* Status Register Bit Designations (AD7190_REG_STAT) */
#define AD7190_STAT_RDY         (1 << 7) // Ready.
#define AD7190_STAT_ERR         (1 << 6) // ADC error bit.
#define AD7190_STAT_NOREF       (1 << 5) // Error no external reference. 
#define AD7190_STAT_PARITY      (1 << 4) // Parity check of the data register. 
#define AD7190_STAT_CH2         (1 << 2) // Channel 2. 
#define AD7190_STAT_CH1         (1 << 1) // Channel 1. 
#define AD7190_STAT_CH0         (1 << 0) // Channel 0. 

/* Mode Register Bit Designations (AD7190_REG_MODE) */
#define AD7190_MODE_SEL(x)      (((x) & 0x7) << 21) // Operation Mode Select.
#define AD7190_MODE_DAT_STA     (1 << 20)           // Status Register transmission.
#define AD7190_MODE_CLKSRC(x)   (((x) & 0x3) << 18)  // Clock Source Select.
#define AD7190_MODE_SINC3       (1 << 15)           // SINC3 Filter Select.
#define AD7190_MODE_ENPAR       (1 << 13)           // Parity Enable.
#define AD7190_MODE_SCYCLE      (1 << 11)           // Single cycle conversion.
#define AD7190_MODE_REJ60       (1 << 10)           // 50/60Hz notch filter.
#define AD7190_MODE_RATE(x)     ((x) & 0x3FF)       // Filter Update Rate Select.

/* Mode Register: AD7190_MODE_SEL(x) options */
#define AD7190_MODE_CONT                0 // Continuous Conversion Mode.
#define AD7190_MODE_SINGLE              1 // Single Conversion Mode.
#define AD7190_MODE_IDLE                2 // Idle Mode.
#define AD7190_MODE_PWRDN               3 // Power-Down Mode.
#define AD7190_MODE_CAL_INT_ZERO        4 // Internal Zero-Scale Calibration.
#define AD7190_MODE_CAL_INT_FULL        5 // Internal Full-Scale Calibration.
#define AD7190_MODE_CAL_SYS_ZERO        6 // System Zero-Scale Calibration.
#define AD7190_MODE_CAL_SYS_FULL        7 // System Full-Scale Calibration.

/* Mode Register: AD7190_MODE_CLKSRC(x) options */
#define AD7190_CLK_EXT_MCLK1_2          0 // External crystal. The external crystal
                                          // is connected from MCLK1 to MCLK2.
#define AD7190_CLK_EXT_MCLK2            1 // External Clock applied to MCLK2 
#define AD7190_CLK_INT                  2 // Internal 4.92 MHz clock. 
                                          // Pin MCLK2 is tristated.
#define AD7190_CLK_INT_CO               3 // Internal 4.92 MHz clock. The internal
                                          // clock is available on MCLK2.

/* Configuration Register Bit Designations (AD7190_REG_CONF) */
#define AD7190_CONF_CHOP        (1 << 23)            // CHOP enable.
#define AD7190_CONF_REFSEL      (1 << 20)            // REFIN1/REFIN2 Reference Select.
#define AD7190_CONF_CHAN(x)     (((x) & 0xFF) << 8)  // Channel select.
#define AD7190_CONF_BURN        (1 << 7)             // Burnout current enable.
#define AD7190_CONF_REFDET      (1 << 6)             // Reference detect enable.
#define AD7190_CONF_BUF         (1 << 4)             // Buffered Mode Enable.
#define AD7190_CONF_UNIPOLAR    (1 << 3)             // Unipolar/Bipolar Enable.
#define AD7190_CONF_GAIN(x)     ((x) & 0x7)          // Gain Select.

/* Configuration Register: AD7190_CONF_CHAN(x) options */
#define AD7190_CH_AIN1P_AIN2M      0 // AIN1(+) - AIN2(-)       
#define AD7190_CH_AIN3P_AIN4M      1 // AIN3(+) - AIN4(-)       
#define AD7190_CH_TEMP_SENSOR      2 // Temperature sensor       
#define AD7190_CH_AIN2P_AIN2M      3 // AIN2(+) - AIN2(-)       
#define AD7190_CH_AIN1P_AINCOM     4 // AIN1(+) - AINCOM       
#define AD7190_CH_AIN2P_AINCOM     5 // AIN2(+) - AINCOM       
#define AD7190_CH_AIN3P_AINCOM     6 // AIN3(+) - AINCOM       
#define AD7190_CH_AIN4P_AINCOM     7 // AIN4(+) - AINCOM

/* Configuration Register: AD7190_CONF_GAIN(x) options */
//                                             ADC Input Range (5 V Reference)
#define AD7190_CONF_GAIN_1		0 // Gain 1    +-5 V
#define AD7190_CONF_GAIN_8		3 // Gain 8    +-625 mV
#define AD7190_CONF_GAIN_16		4 // Gain 16   +-312.5 mV
#define AD7190_CONF_GAIN_32		5 // Gain 32   +-156.2 mV
#define AD7190_CONF_GAIN_64		6 // Gain 64   +-78.125 mV
#define AD7190_CONF_GAIN_128	7 // Gain 128  +-39.06 mV

/* ID Register Bit Designations (AD7190_REG_ID) */
#define ID_AD7190               0x4
#define AD7190_ID_MASK          0x0F

/* GPOCON Register Bit Designations (AD7190_REG_GPOCON) */
#define AD7190_GPOCON_BPDSW     (1 << 6) // Bridge power-down switch enable
#define AD7190_GPOCON_GP32EN    (1 << 5) // Digital Output P3 and P2 enable
#define AD7190_GPOCON_GP10EN    (1 << 4) // Digital Output P1 and P0 enable
#define AD7190_GPOCON_P3DAT     (1 << 3) // P3 state
#define AD7190_GPOCON_P2DAT     (1 << 2) // P2 state
#define AD7190_GPOCON_P1DAT     (1 << 1) // P1 state
#define AD7190_GPOCON_P0DAT     (1 << 0) // P0 state

typedef struct AD7190_SpiDriver_Struct {
  void (*AD7190_CS_Low)(void);
  void (*AD7190_CS_High)(void);
  void (*AD7190_Spi_WriteByte)(uint8_t);
  uint8_t (*AD7190_Spi_ReadByte)(void);
  void (*AD7190_Spi_Write)(uint8_t*, uint16_t);
  void (*AD7190_Spi_Read)(uint8_t*, uint16_t);
  bool (*AD7190_CheckDataReadyPin)(void);
  void (*AD7190_Delay_ms)(uint32_t);
} AD7190_SpiDriver_Typedef;

/******************************************************************************/
/*********************** Functions Declarations *******************************/
/******************************************************************************/

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

#endif /* __AD7190_H__ */
