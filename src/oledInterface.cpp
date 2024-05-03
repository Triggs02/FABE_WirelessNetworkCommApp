#include "oledInterface.h"

#include <iostream>

#include <memory>
#include <array>
#include <stdexcept>

// OLED object to interact with device
SSD1306 my_OLED(OLEDwidth, OLEDheight);  

// Defining a buffer to cover the whole screen
uint8_t screenBuffer[FULLSCREEN];

void initOledScreen()
{
    // Initialize the OLED with corresponding I2C parameters
    const uint16_t I2C_Speed = 0;                    // Library-specific parameter to ensure baudrate remains at 100K
    const uint8_t OLED_I2C_Addr = OLED_I2C_ADDR;     // I2C Address of the OLED device used - peform "i2cdetect -y 1" to verify and change accordingly
    bool I2C_debug = true;                           // Should remain false due to this being an interface for an embedded application

    my_OLED.OLEDbegin(I2C_Speed, OLED_I2C_Addr, I2C_debug);

    // Setting the internaal OLED Buffer pointer for the future
    my_OLED.OLEDSetBufferPtr(OLEDwidth, OLEDheight, screenBuffer, sizeof(screenBuffer));
}


void updateOledScreen(std::string state)
{
    // Collecting the current IP Address of the Raspberry Pi
    std::string cmdIP = "hostname -I | cut -d' ' -f1";
    std::string IP_Addr = "IP: " + exec(cmdIP.c_str());

    // Collecting the current link quality of the Raspberry Pi's network connection
    std::string cmdSignal = "iwconfig wlan0 | grep 'Link Quality' | awk '{print $2}' | cut -d'=' -f2";
    std::string sigQuality = exec(cmdSignal.c_str());   // Expected output: "XX/YY"

    // Quantifying signal strength
    std::string networkStrength;
    if (sigQuality == "")
    {
        networkStrength = "Net: Disconnected";
    } else {
        // Extracting the first part of the signal quality output - from above this would be "XX"
        int currStrength = std::stoi(sigQuality.substr(0, sigQuality.find("/")));

        if (currStrength < 20)
        {
            networkStrength = "Net: Weak";
        } else if (currStrength < 50)
        {
            networkStrength = "Net: Average";
        } else {
            networkStrength = "Net: Strong";
        }
    }

    // Collecting the current internal temperature of the Raspberry Pi
    std::string cmdTemp = "vcgencmd measure_temp | cut -f2 -d'='";
    std::string currTemp = "Temp: " + exec(cmdTemp.c_str());

    // Ensuring the slave address of the OLED is set
    bcm2835_i2c_setSlaveAddress(OLED_I2C_ADDR);

    my_OLED.setFontNum(OLEDFont_Default);
    my_OLED.OLEDclearBuffer();

    // Displaying the collected status values of the system on the OLED
    my_OLED.setCursor(0, 0);
    my_OLED.print(IP_Addr.c_str());

    my_OLED.setCursor(0, 16);
    my_OLED.print(networkStrength.c_str());

    my_OLED.setCursor(0, 32);
    my_OLED.print(currTemp.c_str());

    my_OLED.setCursor(0, 48);
    my_OLED.print(state.insert(0, "State: ").c_str());

    my_OLED.OLEDupdate();
}


// Implementation for this function was taken from:
// https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po
std::string exec(const char * cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


// int main() {

//     std::cout << "Beginning program" << std::endl;

//     bcm2835_init();

//     my_OLED.OLED_I2C_ON();

//     initOledScreen();
//     updateOledScreen("Unknown");

//     return 0;
// }

// void updateOledScreen(int state)
// {
//     std::stringstream command;

//     // Initializing the command to call the corresponding python file
//     command << "python3 ~/github/Repos/ECE3905-Team4-PugComms/mergedCode/stats.py " << state;

//     // Calling the python function
//     system(command.str().c_str());
// }