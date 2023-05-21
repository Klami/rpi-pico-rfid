#include "MFRC522.h"
#include "hardware/spi.h"

#ifndef SCK_PIN
#error SCK_PIN not set
#endif
#ifndef MOSI_PIN
#error MOSI_PIN not set
#endif
#ifndef MISO_PIN
#error MISO_PIN not set
#endif
#ifndef SS
#error SS/CS not set
#endif

void MFRC522::SPI_INIT()
{
    spi_init(spi1, MFRC522_SPICLOCK);

    spi_set_format(spi1,       // SPI instance
                   8,          // Number of bits per transfer
                   SPI_CPOL_1, // Polarity (CPOL)
                   SPI_CPHA_1, // Phase (CPHA)
                   SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
}

/**
 * Writes a  uint8_t to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
                                uint8_t value     ///< The value to write.
)
{
    gpio_put(_chipSelectPin, 0); // Select slave
    const uint8_t regnum = reg;
    spi_write_blocking(spi0, &regnum, 1); // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    spi_write_blocking(spi0, &value, 1);
    gpio_put(_chipSelectPin, 1); // Release slave again
} // End PCD_WriteRegister()

/**
 * Writes a number of  uint8_ts to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
                                uint8_t count,    ///< The number of  uint8_ts to write to the register
                                uint8_t *values   ///< The values to write.  uint8_t array.
)
{
    gpio_put(_chipSelectPin, 0); // Select slave
    const uint8_t regnum = reg;
    spi_write_blocking(spi0, &regnum, 1); // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    for (uint8_t index = 0; index < count; index++)
    {
        spi_write_blocking(spi0, &values[index], 1);
    }
    gpio_put(_chipSelectPin, 1); // Release slave again
} // End PCD_WriteRegister()

/**
 * Reads a  uint8_t from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t MFRC522::PCD_ReadRegister(PCD_Register reg ///< The register to read from. One of the PCD_Register enums.
)
{
    uint8_t value;
    gpio_put(_chipSelectPin, 0);
    const uint8_t regnum = reg | 0x80;                 // Select slave
    spi_write_read_blocking(spi0, &regnum, &value, 1); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    spi_read_blocking(spi0, 0xFF, &value, 1);          // Read the value back. Send 0 to stop reading.
    gpio_put(_chipSelectPin, 1);                       // Release slave again
    return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of  uint8_ts from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_ReadRegister(PCD_Register reg, ///< The register to read from. One of the PCD_Register enums.
                               uint8_t count,    ///< The number of bytes to read
                               uint8_t *values,  ///<  uint8_t array to store the values in.
                               uint8_t rxAlign   ///< Only bit positions rxAlign..7 in values[0] are updated.
)
{
    if (count == 0)
    {
        return;
    }
    // printf("Reading ")); 	Serial.print(count); Serial.println(F("  uint8_ts from register.");
    uint8_t address = 0x80 | reg;                           // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t value = values[0];                              // Index in values array.
    gpio_put(_chipSelectPin, 0);                            // Select slave
    count--;                                                // One read is performed outside of the loop
    spi_write_read_blocking(spi0, &address, values, count); // Tell MFRC522 which address we want to read
    if (rxAlign)
    { // Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (value & ~mask) | (values[0] & mask);
    }
    gpio_put(_chipSelectPin, 1); // Release slave again
} // End PCD_ReadRegister()