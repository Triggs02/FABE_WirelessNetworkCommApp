#pragma once

#ifndef RELAY_INTERFACE_H
#define RELAY_INTERFACE_H

#include <bcm2835.h>

// Defining important parameters to control whether the relay is turned ON or OFF
#define ON 0xFF
#define OFF 0x00

// Defining an enum that must be used to access a specific relay channel
enum relayChannelNum {NO_1 = 1, NO_2 = 2, NO_3 = 3, NO_4 = 4};

// Defining possible I2C addresses of the relay
#define ADDR_1 0x10     // This is the default address
#define ADDR_2 0x11
#define ADDR_3 0x12
#define ADDR_4 0x13

// Defining recommended delay period after command is sent (in milliseconds)
#define WAIT_PERIOD 100

/**
 * Purpose: Write the specified state, ON or OFF, to the specified relay channel at the given I2C address.
 *          Does not preserve the previous I2C address being used for comms (if any), so the user will need
 *          to make sure that previous I2C address is properly saved and restored.
 *          Assumes the bcm2835 library has already been initialized and baudrate has been set to standard 100 KHz.
 * Returns: Nothing
*/
void write_i2c_relayRegister(uint8_t relayI2C_Addr, relayChannelNum num, uint8_t relayState);

#endif