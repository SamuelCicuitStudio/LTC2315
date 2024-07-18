#include "Function.h"


void setup() {
  Serial.begin(115200);
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  LCT2315SPIBegin();
  AD9833Begin();
  PulseCounterInit();
}

void loop() {
  /**************************************************************************************/
  //Read LTC2315 values
  //float adc_voltage = readADCLTC2315();
  //Serial.println(adc_voltage, 3);
  /**************************************************************************************/
  // Get pulse frequency
  //int16_t frequency = GetPulseFrequency();
  // Print frequency to serial monitor
  //Serial.print("Frequency: ");
  //Serial.print(frequency);
  //Serial.println(" Hz");
  //delay(1000); // Delay for 1 second
  /**************************************************************************************/
  // use the AD9833
  //setFrequency(0, 1000.0);  // Set frequency to 1000 Hz on Register 0
  //setWaveform(0, TRIANGLE_WAVE);   // Set waveform to Triangle Wave on Register 0
  //enableOutput(true);       // Enable output
  //delay(5000);  // Wait for 5 seconds
  //setFrequency(0, 5000.0);  // Change frequency to 5000 Hz on Register 0
  //setWaveform(0, SQUARE_WAVE);   // Change waveform to Square Wave on Register 0
  //delay(5000);  // Wait for 5 seconds
  //enableOutput(false);       // Disable output

}

