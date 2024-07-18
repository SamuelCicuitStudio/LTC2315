#include "Vars.h"
/***********************************************************************************************************************************************************************/
// Define HSPI pins on ESP32
#define LTC2315_CS_PIN 15    // GPIO pin for Chip Select (CS)
#define LTC2315_SCK_PIN 14   // GPIO pin for SPI Clock (SCK)
#define LTC2315_MISO_PIN 12  // GPIO pin for SPI MISO
#define LTC2315_MOSI_PIN 13  // PIO pin for SPI MOSI
/***********************************************************************************************************************************************************************/
// Define VSPI pins on ESP32
#define AD9833_HSPI_SCK 18 // HSPI SCK (Clock)
#define AD9833_HSPI_MOSI 23 // HSPI MOSI (Master Out Slave In)
#define AD9833_HSPI_MISO 19 // HSPI MISO (Master In Slave Out)
#define AD9833_CS_PIN 5 // Chip select pin for AD9833

/***********************************************************************************************************************************************************************/
#define PulseCounterPin  21  // Change to your desired input pin for PCNT

