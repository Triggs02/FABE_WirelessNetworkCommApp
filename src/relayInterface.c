#include "relayInterface.h"


void write_i2c_relayRegister(uint8_t relayI2C_Addr, relayChannelNum num, uint8_t relayState)
{
    // Setting the selected relay's I2C address
    bcm2835_i2c_setSlaveAddress(relayI2C_Addr);

    // Initiating command to be sent that includes the relay channel number & state to be written
    const char command[2] = {num, relayState};

    // Sending the command
    bcm2835_i2c_write(command, 2);

    // Delaying a brief amount to ensure command has been received
    bcm2835_delay(WAIT_PERIOD);
}
