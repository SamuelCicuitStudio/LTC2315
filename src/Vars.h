
#include <SPI.h>
#include <Arduino.h>
#include <driver/pcnt.h>
/***********************************************************************************************************************************************************************/

static uint8_t LTC2315_shift = 2;
float LTC2315_vref = 3.318;
/***********************************************************************************************************************************************************************/

// AD9833 Registers and Commands
#define FREQ0_WRITE_REG 0x4000
#define FREQ1_WRITE_REG 0x8000
#define PHASE_WRITE_CMD 0xC000
#define RESET_CMD 0x0100
#define DISABLE_DAC 0x0010
#define DISABLE_INT_CLK 0x0080

// Waveform Types
enum WaveformType {
  SINE_WAVE,
  TRIANGLE_WAVE,
  SQUARE_WAVE,
  HALF_SQUARE_WAVE
};

uint32_t refFrequency = 25000000; // Reference frequency of AD9833
/***********************************************************************************************************************************************************************/
// SPISettings for LTC2315 on VSPI
SPISettings ltcSettings(5000000, MSBFIRST, SPI_MODE0);
// SPISettings for AD9833 on HSPI
SPISettings ad9833Settings(2000000, MSBFIRST, SPI_MODE2);

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;
