#include "Pins.h"

/***********************************************************************************************************************************************************************/
// Function prototypes
// Initializes SPI communication for the LTC2315 ADC
void LCT2315SPIBegin();                                                                                           // Begins HSPI for LTC2315
// Reads the ADC value from the LTC2315 and returns it as a float
float readADCLTC2315();                                                                                           // Reads ADC value from LTC2315
// Initializes the Pulse Counter (PCNT) module
void PulseCounterInit();                                                                                          // Configures and initializes PCNT module
// Resets the Pulse Counter
void ResetPulseCount();                                                                                           // Resets PCNT counter
// Pauses the Pulse Counter
void PausePulseCount();                                                                                           // Pauses PCNT counter
// Resumes the Pulse Counter
void ResumePulseCount();                                                                                          // Resumes PCNT counter
// Returns the current pulse frequency as an integer
int16_t GetPulseFrequency();                                                                                      // Reads current counter value to calculate frequency
// Initializes SPI communication for the AD9833 waveform generator
void AD9833Begin();                                                                                               // Begins VSPI for AD9833
// Sets the frequency of the AD9833 waveform generator
void setFrequency(uint16_t reg, float frequency);                                                                 // Sets frequency of AD9833
// Sets the waveform type for the AD9833 (SINE, TRIANGLE, SQUARE, HALF_SQUARE)
void setWaveform(uint16_t reg, WaveformType waveType);                                                            // Sets waveform type for AD9833
// Enables or disables the output of the AD9833
void enableOutput(bool enable);                                                                                   // Enables or disables AD9833 output
// Writes a 16-bit data to the AD9833 register via SPI
void writeRegister(uint16_t dat);                                                                                 // Writes data to AD9833 register
/***********************************************************************************************************************************************************************/
void LCT2315SPIBegin(){
  // Begin HSPI for LTC2315
  vspi->begin(LTC2315_SCK_PIN, LTC2315_MISO_PIN, LTC2315_MOSI_PIN, LTC2315_CS_PIN);
}
/***********************************************************************************************************************************************************************/

float readADCLTC2315() {
  uint16_t adc_code;
  uint8_t b[2];
  
  vspi->beginTransaction(ltcSettings);
  digitalWrite(LTC2315_CS_PIN, LOW);
  b[1] = vspi->transfer(0x00);  // Read MSB and send dummy byte
  b[0] = vspi->transfer(0x00);  // Read LSB and send dummy byte
  digitalWrite(LTC2315_CS_PIN, HIGH);
  vspi->endTransaction();
  
  adc_code = ((uint16_t)b[1] << 8) | b[0];
  adc_code <<= LTC2315_shift;

  return ((float)adc_code / ((1 << 16) - 1)) * LTC2315_vref;
}

/***********************************************************************************************************************************************************************/
void PulseCounterInit(){
      // Configure and initialize PCNT module
  pcnt_config_t pcntConfig;
  pcntConfig.pulse_gpio_num = PulseCounterPin;
  pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcntConfig.channel = PCNT_CHANNEL_0;
  pcntConfig.unit = PCNT_UNIT_0;
  pcntConfig.pos_mode = PCNT_COUNT_INC;    // Count rising edges
  pcntConfig.neg_mode = PCNT_COUNT_DIS;    // Don't count falling edges
  pcntConfig.lctrl_mode = PCNT_MODE_KEEP;  // Keep counting on low level
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;  // Keep counting on high level
  pcntConfig.counter_h_lim = 0xFFFF;       // Set the maximum counter value

  // Initialize PCNT unit
  pcnt_unit_config(&pcntConfig);
  pcnt_counter_pause(PCNT_UNIT_0);  // Pause counter while configuring
  pcnt_counter_clear(PCNT_UNIT_0);  // Clear counter value to zero
  pcnt_counter_resume(PCNT_UNIT_0); // Resume counter
}
/***********************************************************************************************************************************************************************/
void ResetPulseCount(){
     // Reset PCNT counter
    pcnt_counter_pause(PCNT_UNIT_0);  // Pause counter before clearing
    pcnt_counter_clear(PCNT_UNIT_0);  // Clear counter value to zero
    pcnt_counter_resume(PCNT_UNIT_0); // Resume counter after clearing
    Serial.println("Counter reset.");
}
/***********************************************************************************************************************************************************************/
void PausePulseCount(){
     // Reset PCNT counter
    pcnt_counter_pause(PCNT_UNIT_0);  // Pause counter before clearing
    Serial.println("Counter Paused.");
}
/***********************************************************************************************************************************************************************/
void ResumePulseCount(){
    // Reset PCNT counter
    pcnt_counter_resume(PCNT_UNIT_0); // Resume counter after clearing
    Serial.println("Counter resumed.");
}
/***********************************************************************************************************************************************************************/
int16_t  GetPulseFrequency(){
      // Read current counter value
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  // Variables to store frequency
  float frequency = 0.0;
  // Calculate frequency based on counter value over 1 second
  frequency = (float)count; // Assuming each count is a pulse per second
  
  // Print frequency to serial monitor
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");
  return frequency;
}
/***********************************************************************************************************************************************************************/
void AD9833Begin() {
  // Initialize SPI using defined pins for VSPI (HSPI: SCK, MOSI, MISO)
  hspi->begin(AD9833_HSPI_SCK, AD9833_HSPI_MISO, AD9833_HSPI_MOSI, AD9833_CS_PIN);
  delay(100);
}
/***********************************************************************************************************************************************************************/
void setFrequency(uint16_t reg, float frequency) {
  if (frequency < 0.0) frequency = 0.0;
  if (frequency > 12500000.0) frequency = 12500000.0; // Maximum frequency limit

  uint32_t freqWord = frequency * (1 << 28) / refFrequency;
  uint16_t regCmd = (reg == 0) ? FREQ0_WRITE_REG : FREQ1_WRITE_REG;
  uint16_t lower14 = regCmd | (freqWord & 0x3FFF);
  uint16_t upper14 = regCmd | ((freqWord >> 14) & 0x3FFF);

  writeRegister(lower14);
  writeRegister(upper14);
}
/***********************************************************************************************************************************************************************/
void setWaveform(uint16_t reg, WaveformType waveType) {
  uint16_t regCmd = (reg == 0) ? FREQ0_WRITE_REG : FREQ1_WRITE_REG;
  uint16_t controlReg = regCmd;

  switch (waveType) {
    case SINE_WAVE:
      break;
    case TRIANGLE_WAVE:
      controlReg |= 0x2000; // Bit D1 set for triangle wave
      break;
    case SQUARE_WAVE:
      controlReg |= 0x2028; // Bits D1 and D5 set for square wave
      break;
    case HALF_SQUARE_WAVE:
      controlReg |= 0x2008; // Bits D1 and D3 set for half square wave
      break;
    default:
      return; // Invalid waveform type
  }

  writeRegister(controlReg);
}
/***********************************************************************************************************************************************************************/
void enableOutput(bool enable) {
  uint16_t controlReg = enable ? 0x0000 : RESET_CMD;
  writeRegister(controlReg);
}
/***********************************************************************************************************************************************************************/
void writeRegister(uint16_t dat) {
  digitalWrite(AD9833_CS_PIN, LOW); // Set AD9833 CS pin LOW to enable SPI communication

  hspi->beginTransaction(ad9833Settings);
  hspi->transfer16(dat); // Transfer 16 bits of data
  hspi->endTransaction();

  digitalWrite(AD9833_CS_PIN, HIGH); // Disable SPI communication by setting AD9833 CS pin HIGH
}
