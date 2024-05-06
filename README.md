# ECE 4905 - Team Wireless Pug Communications Wireless Communication System Project Repository

## Software Setup Guide

Guide on how to properly build the code contained within this repository for the Raspberry Pi 4B system it is intended to run on.

### Required Software

* It is recommended to use Microsoft's Visual Studio Code to remotely access the Raspberry Pi for development purposes (VSCode's 'Remote - SSH' extension); however, using the built-in Desktop of the Raspberry Pi will also work for both the use of this software program and future code development.
* The most recent version of the Arduino IDE (to build and flash an Arduino to use the mock sensor framework)
* **Before executing the WirelessNetworkApp**: Make sure that the Arduino is flashed with the 'mockSensor.ino' program and corresponding communication selection switch are properly connected to the Raspberry Pi's GPIO pins.  The details for the circuit connection can be found under the 'Doc/' sub-directory.

### The Mock Sensor Program 'mockSensor.ino'

* Required Hardware: An Arduino Uno R3 board (or any equivalent board with an Atmega328p)
* Required Software: The most up-to-date version of the Arduino IDE
* Required Files: The 'mockSensor.ino' file located in the 'MockSensor/' sub-directory of this repo.

Steps to Properly Build and Run the Mock Sensor:

1. Open the MockSensor/mockSensor.ino project in the Arduino IDE on a Windows computer. 
        - As of now, the Arduino IDE version 2.0 is not compatible with the Raspberry Pi platform; therefore, it is strongly  recommended to put this file on a Windows computer and build/flash the Arduino that way.
2. Configure the project to use the Arduino board of interest.
3. Press the 'Upload' button with the Arduino connnected.

Details on how the Mock Sensor program operates have been provided in the 'Doc/' sub-directory.

### The Raspberry Pi 'wirelessNetworkApp'

* Required Hardware: Raspberry Pi, SSR RPi Hat connected, all other hardware required to run the program successfully connected to the Raspberry Pi's GPIO pins.
* Required Software: Linux Bash Terminal
* Required Files: All files contained within the Github

Steps to Properly Build and Run the WirelessNetworkApp:

1. Clone the 'FABE_WirelessNetworkCommApp' repository from Github and then enter it by issuing the following commands in the Linux command line from where you intend to use the code:
   
```bash
git clone https://github.com/Triggs02/FABE_WirelessNetworkCommApp.git
cd FABE_WirelessNetworkCommApp
```

2. Ensure that the 'install_libraries.sh' script is executable by issuing the following command in the Linux terminal.  The file is considered executable on the Raspberry Pi when the file name appears green.

```bash
chmod +x install_libraries.sh
```

3. Run the bash shell 'install_libaries.sh' to install all necessary dependencies used to make the 'WirelessNetworkApp' run properly.  Two sub-directories titled "Data_Collected_Offline" and "Temp_Data_Collected" should have been created after running this script.  If they have not been, then they must be created in the root directory.

```bash
./install_libraries.sh
```

4. Ensure you are currently in the root directory of the repository and then run the Makefile present in the root directory to build the 'WirelessNetworkApp - the command required to run the Makefile has been provided below.  This should result in two additional sub-directories created: obj/ and build/.

```bash
make
```

5. After running the Makefile, the built executable should be present in the 'bin/' sub-directory. Navigating there and running the executable should result in a working program (**All Hardware Must Be Connected**).  The program expects the user to specify the communication type (either an 'I' for I2C or 'S' for Serial (UART)) as an argument to the executable.  In this case, I2C communications are what has been fully tested by the team; therefore, the value 'I' should be supplied.  The program is required to be run as a sudo user due to the bcm2835 libraries' requirements.

```bash
cd bin/
sudo ./wirelessNetworkApp I
```

6. (OPTIONAL) To clean up the obj/ and build/ folders from the working directory, issue the following Make command in the Linux terminal:

```bash
make clean
```

### Additional Files in the 'extraCode' Folder

The additional files present in the 'extraCode' folder were those used in the attempt to develop for the OLED screen part of the project. They have been provided for the user's convenience if they want to try and add this feature to the project.  It is strongly recommended to use the C++ version of the OLED interaction interface that has been provided in the 'src/' sub-directory.
