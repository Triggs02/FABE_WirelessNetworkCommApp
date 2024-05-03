#pragma once

#ifndef OLED_INTERFACE_H
#define OLED_INTERFACE_H

#include "SSD1306_OLED.hpp"     // 3rd party library used to interact with OLED screen
                                // Taken from here: https://github.com/gavinlyonsrepo/SSD1306_OLED_RPI/tree/main

#include <string>

// Important OLED interaction variables
#define OLEDwidth 128
#define OLEDheight 64
#define FULLSCREEN (OLEDwidth * (OLEDheight / 8))
#define OLED_I2C_ADDR 0x3C

/**
 * Purpose: (Only call once in a program) Initialize the OLED screen to 
 *          communicate properly with the Raspberry Pi over I2C.  Assumes that 
 *          the I2C bus has already been initialized using the bcm2835 library. 
 *          Will not work properly otherwise. Also, after calling this the user 
 *          will need to re-initialize any previous slave address that was set in the bcm2835 library.
 * Returns: Nothing
*/
void initOledScreen();

/**
 * Purpose: Used to update the OLED screen with the current state
 *          and related system parameters of the RPi.
*/
void updateOledScreen(std::string state);

// Helper functions to improve ease of issuing
// and receiving data from system commands:

/**
 * Purpose: Used to execute the specified <cmd> in the Linux command
 *          terminal and collect the terminal's response from stdout.
 * Returns: A string containing the output of the executed Linux command.
*/
std::string exec(const char* cmd);

#endif