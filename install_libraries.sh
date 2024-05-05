#!/bin/bash

# Function to check if a library is installed
check_library() {
    if [ -f "$1" ]; then
        echo "Library $2 is installed."
    else
        echo "Library $2 is not installed. Installing..."
        return 1
    fi
}

# Check and install bcm2835 library
check_library "/usr/local/include/bcm2835.h" "bcm2835.h" && \
check_library "/usr/local/lib/libbcm2835.a"  "libbcm2835.a" || {
    mkdir bcm2835_install
    cd bcm2835_install
    wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.75.tar.gz
    tar zxvf bcm2835-1.75.tar.gz
    cd bcm2835-1.75/
    sudo ./configure && sudo make && sudo make check && sudo make install
    cd ..
    rm -rf bcm2835-1.75.tar.gz bcm2835-1.75/
    cd ..
}

# Check and install SSD1306_OLED_RPI library
check_library "/usr/lib/libSSD1306_OLED_RPI.so" "libSSD1306_OLED_RPI.so" && \
check_library "/usr/lib/libSSD1306_OLED_RPI.so.1" "libSSD1306_OLED_RPI.so.1" && \
check_library "/usr/include/SSD1306_OLED_font.hpp" "SSD1306_OLED_font.hpp" && \
check_library "/usr/include/SSD1306_OLED_graphics.hpp" "SSD1306_OLED_graphics.hpp" && \
check_library "/usr/include/SSD1306_OLED.hpp" "SSD1306_OLED.hpp" && \
check_library "/usr/include/SSD1306_OLED_Print.hpp" "SSD1306_OLED_Print.hpp" || {
    mkdir SSD1306_OLED_RPI_install
    cd SSD1306_OLED_RPI_install
    curl -sl https://github.com/gavinlyonsrepo/SSD1306_OLED_RPI/archive/1.6.1.tar.gz | tar xz
    cd SSD1306_OLED_RPI-1.6.1
    make && sudo make install
    cd ../..
    rm -rf SSD1306_OLED_RPI-1.6.1
}

# Generate two sub-directories if they don't exist
if [ ! -d "Data_Collected_Offline" ]; then
    mkdir "Data_Collected_Offline"
    echo "Data_Collected_Offline directory created."
fi

if [ ! -d "Temp_Data_Collected" ]; then
    mkdir "Temp_Data_Collected"
    echo "Temp_Data_Collected directory created."
fi
