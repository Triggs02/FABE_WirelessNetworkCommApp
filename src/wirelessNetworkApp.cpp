#include <bcm2835.h>
#include <chrono>           // Used to obtain most up-to-date system time information
#include <iostream>
#include <iomanip>
#include <fstream>
#include <signal.h>
#include <string>
#include <sstream>          // Used to create and manipulate file names
#include <sys/socket.h>     // 
#include "string.h"         // Used for 'memcpy'
#include "rs232.h"          // Custom library found online @ https://gitlab.com/Teuniz/RS-232.git that will be used to implement - Arduino is on ttyACM1 which is comports[25]

#include "network_Credentials.h"    // Used to securely hold network credentials used for wireless comms
#include "relayInterface.h"        // Used to properly interact with the SSR relay attachment board that is on the RPi
#include "oledInterface.h"         // Used to properly interact with the informational OLED

using namespace std;

// Defining avaible I2C sensor controller commands for the RPi to use
/**
 * The commands are as follows:
 *    - 0x01 --> Read_Device_Status
 *    - 0x02 --> Begin_Data_Collection
 *    - 0x03 --> Collect_Data
 *    - 0x04 --> Terminate_Data_Collection
 */
#define READ_DEVICE_STATUS_CMD      0x01
#define BEG_DATA_COLLECT_CMD        0x02
#define COLLECT_DATA_CMD            0x03
#define TERMINATE_DATA_COLLECT_CMD  0x04

// Defining expected max buffer size (in bytes) parameters for each command
#define READ_DEVICE_STATUS_MSG_SIZE 1
#define BEG_DATA_COLLECT_MSG_SIZE   1
#define COLLECT_DATA_MSG_SIZE       32
#define TERM_DATA_COLLECT_MSG_SIZE   1

// Defining delay values after sending commands and before reading the result of the command (all are in milliseconds)
#define READ_DEVICE_STATUS_DELAY    100
#define BEG_DATA_COLLECT_DELAY      100
#define COLLECT_DATA_DELAY          175
#define SENSOR_CHECK_RETRY_PERIOD   100
#define TERM_DATA_COLLECT_DELAY     100

// Defining data collection delay (defining the sampling period - in milliseconds)
#define DATA_SAMPLING_PERIOD 500

// Defining the period at which the RPi checks that it is on - in milliseconds
#define RPI_ON_CHECK_PERIOD 1500

// Defining important GPIO pins
#define PIN RPI_GPIO_P1_18

// Defining waiting period upper threshold for checking if the sensor is "on"
#define MAX_RETRIES 5
// Defining waiting period upper threshold for failed comm attempts w/ sensor
#define MAX_COMM_FAIL_RETRIES 10

// Defining data collection parameters
size_t maxNumBytesToCollect = 1024; // 1 KB
// Counter used to ensure no more than the max # bytes are collected
size_t numBytesCollectAmount = 0;    
string tempDataCollectionFolderName = "Temp_Data_Collected";
string dataCollectionFolderName = "Data_Collected_Offline";

// Defining internal states of RPi for its sensor data collection lifecycle
//                      S0           S1                S2                  S3              S4            S5           S6            S7         
enum mockSensorStates {SLEEP, CHECK_SENSOR_ON, WAIT_FOR_RETRY_CHECK, INIT_DATA_MEASURE, COLLECT_DATA, DATA_PACKAGE, TRANSMIT_DATA, DATA_STORE} state;

// Defining important simulation values
bool dataMeasurementStarted = false;        // Used to indicate whether a start measurement command has been issued already.
                                            // Will be used to determine when a start measurement command can be sent.
bool dataTerminatedError = false;           // Used to indicate whether the RPi should continue trying to terminate the current data collection process or not                                            
enum dataCommType {SERIAL, I2C} dataComm;   // Used to indicate what data communications type will be used for communicating with the sensor

// Defining signal atomic value to allow for graceful program closeure upon reciving a Ctrl+C while running
volatile sig_atomic_t stopProgram;

// Defining important Serial comms parameters
int arduinoSerialPortNum = 25;  // Used to index into comports list for rs232.h libray
int arduinoBaudRate = 9600;     // Same baud rate that will be used on the Arduino's end
char serialMode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit

// Utility Function Protoypes
void sigIntHandler(int signum);
bool interpretBCMErrCode(uint8_t code);
stringstream generateNextFileName(const string & folderName, int fileNum);
stringstream generateFileNameForDifferentFolder(const string & prevFileName, const string & newFolderName);
std::string createShortenedStateName(mockSensorStates state);

// State Handling Function Prototypes
bool sendReadDeviceStatusCommand();
bool sendBegDataCollectCommand(uint8_t & data);
bool sendCollectDataCommand(uint8_t * data);
bool sendTerminateDataCollectCommand();
bool formatData(ofstream & rawData);
bool saveDataLocally(const string& newFileName, const string& tempFileName);

// Wireless Communication Function Prototypes
bool isConnectedToInternet();
bool transmitData(const string& fileName, const string& userName, const string& password, const string& ipAddress, const string& fileDestination);
void processData(int fileNumber, const string& userName, const string& password, const string& ipAddress, const string& fileDestination);

int main(int argc, char *argv[]) {

    uint8_t arduinoID = 2;              // Defining the "sensor"'s i2c addr
    int sensorCheckRetryCounter = 0;    // Used to ensure the RPi does not continuously loop when trying to retry the sensor pre-check
    int failCommSensorCounter = 0;      // Used to keep track of within the process of each state to ensure RPi does not continuously check
                                        // for data coming from the mock sensor.

    // Initializing the signal handler to the SIGINT signal (Ctrl^C)
    signal(SIGINT, sigIntHandler);

    // Running through available 'user' commands to configure
    // internal comm parameters
    char commType;  // Defining the communication type to be collected
    if (argc == 2)
    {
        commType = *argv[1];

        if (commType == 'S') {
        // Initialize Serial comms only
        cout << "Initializing Serial comms!" << endl << endl;

        if (RS232_OpenComport(arduinoSerialPortNum, arduinoBaudRate, serialMode, 0)) {
            cout << "The serial port is already opened! Aborting program. Please try again." << endl;
            return 2;
        } else {
            // Setting the internal comm state to ensure I2C is not used
            dataComm = SERIAL;
        }

        } else if (commType == 'I') {
            // Initializing i2c comms only
            cout << "Initializing i2c comms!" << endl << endl;

            if (!bcm2835_init()) {
                return 1;
            } else {
                // Setting the internal comm state to ensure Serial is not used
                dataComm = I2C;
            }
            bcm2835_i2c_begin();                        // Starting i2C operations
            bcm2835_i2c_setSlaveAddress(arduinoID);     // Setting the i2C address of the Arduino
            bcm2835_i2c_set_baudrate(100000);           // Setting a 100K baudrate (standard) for i2C communications - translates to 100 KHz I2C bus frequency

        } else {
            // Initializing i2c comms by default
            cout << "Could not translate char entered. Setting up i2c comms by default." << endl;

            if (!bcm2835_init()) {
                return 1;
            } else {
                // Setting the internal comm state to ensure Serial is not used
                dataComm = I2C;
            }
            bcm2835_i2c_begin();                        // Starting i2C operations
            bcm2835_i2c_setSlaveAddress(arduinoID);     // Setting the i2C address of the Arduino
            bcm2835_i2c_set_baudrate(100000);           // Setting a 100K baudrate (standard) for i2C communications - translates to 100 KHz I2C bus frequency
        }
    } else {
        cout << "Did not supply the correct argument. Supply either an 'I' or 'S'" << endl;
        return 1;
    }

    // Configuring a GPIO pin PIN to serve as the pulse detector to get the device out of its SLEEP state
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
    // Setting the pin to have a pullup resistor
    bcm2835_gpio_set_pud(PIN, BCM2835_GPIO_PUD_UP);
    // Setting the pin to be low detect enable
    bcm2835_gpio_len(PIN);

    // Declaring the necessary components of the temporary data file that collects 
    // each chunk of data before being sent wirelessly or stored locally
    ofstream *tempFile;
    stringstream tempDataCollectFile_Name; // Used to keep track of the temp file name for each collection chunk
    int dataCollectionNum = 0;


    // Entering the system state machine - ensuring the RPi is initially in 'Sleep' to conserve power
    state = SLEEP;

    while (!stopProgram) {
        switch(state)
        {   
            // State S0
            case SLEEP:
            {
                if (bcm2835_gpio_eds(PIN))
                {

                    // Clearing the eds flag to prepare for next time RPi enters sleep mode
                    bcm2835_gpio_set_eds(PIN);

                    cout << "The RPi is on!" << endl;

                    // Turn on the necessary relays on the SSR hat connected to the RPi
                    write_i2c_relayRegister(ADDR_1, NO_1, ON);      // Turning on the mock sensor
                    write_i2c_relayRegister(ADDR_1, NO_2, ON);      // Turning on the OLED screen

                    // For FABE: Leave this uncommented until the OLED has been completely implemented.
                    //           Please explore the interface designed for interacting the OLED
                    // initOledScreen();

                    // Setting the slave addr back to the sensors
                    bcm2835_i2c_setSlaveAddress(arduinoID);

                    // Delaying to allow for change-over
                    bcm2835_delay(200);

                    // Determining if a terminate command should be sent in an attempt to reset the mock sensor
                    if (dataTerminatedError)
                    {
                        if (sendTerminateDataCollectCommand())
                        {
                            dataTerminatedError = false;
                        }
                    }

                    // Now that RPi has received an "On" signal, it should go to check the sensor
                    state = CHECK_SENSOR_ON;
                } else {
                    cout << "The RPi is off" << endl;

                    // Turning off the necessary relays on the SSR hat connected to the RPi
                    write_i2c_relayRegister(ADDR_1, NO_1, OFF);   // Turning off the mock sensor
                    write_i2c_relayRegister(ADDR_1, NO_2, OFF);   // Turning off the OLED screen

                    // Setting the slave addr back to the sensors
                    bcm2835_i2c_setSlaveAddress(arduinoID);

                    // Delaying to allow for the change-over
                    bcm2835_delay(200);

                    state = SLEEP;
                }
                // Waiting for a period of time before checking the RPis state again
                bcm2835_delay(RPI_ON_CHECK_PERIOD);

                break;
            }
            // State S1
            case CHECK_SENSOR_ON:
            {
                cout << endl << "You are now in the Check_Sensor_On state" << endl;
                if (sendReadDeviceStatusCommand()) {
                    cout << "The Read_Device_Status command worked successfully." << endl;
                    state = INIT_DATA_MEASURE;
                } else {
                    cout << "The Read_Device_Status command failed. Please debug me!" << endl;
                    state = WAIT_FOR_RETRY_CHECK;
                }

                // Causing slight delay to ensure a lockup doesn't occur
                bcm2835_delay(250);
                break;
            }
            // State S2
            case WAIT_FOR_RETRY_CHECK:
            {
                cout << endl << "You are now in the Wait_for_Retry_Check state" << endl;
                
                // Determine if the upper threshold of retries has been reached before trying again
                if (sensorCheckRetryCounter < MAX_RETRIES)
                {
                    // Wait for the specified period and then attempt to check the sensor again
                    cout << "Waiting for " << SENSOR_CHECK_RETRY_PERIOD << " milliseconds before trying to check the sensor again." << endl;
                    cout << "This is retry #" << sensorCheckRetryCounter + 1 << endl;
                    bcm2835_delay(SENSOR_CHECK_RETRY_PERIOD);
                    state = CHECK_SENSOR_ON;

                    sensorCheckRetryCounter++;
                } else {
                    // Resetting the counter and ensuring the RPi is able to sleep again
                    sensorCheckRetryCounter = 0;
                    state = SLEEP;
                }
                break;
            }
            // State S3
            case INIT_DATA_MEASURE:
            {
                cout << endl << "You are now in the Poll_for_Data state" << endl;
                if (!dataMeasurementStarted) {
                    uint8_t data;   // Used to collect measurement status from sensor

                    if (sendBegDataCollectCommand(data)) {
                        // Data collection was started successfully, so preparing the temporary data storage for the data
                        if (data == 1) {
                            tempDataCollectFile_Name = generateNextFileName(tempDataCollectionFolderName, dataCollectionNum);

                            // Initializing temporary data collection file
                            tempFile = new ofstream(tempDataCollectFile_Name.str());

                            // Only proceeding w/ data measurement if the collection file was initialized
                            if (tempFile->is_open()) {
                                dataMeasurementStarted = true;
                                failCommSensorCounter = 0;  // Resetting the failed communications counter

                                cout << "Data measurement has been started successfully!" << endl;

                                // Directing the RPi to continuously poll the sensor for data as it comes in
                                state = COLLECT_DATA;
                            } else {
                                dataMeasurementStarted = false;

                                cout << "The sensor was able to init the data measurement but the RPi could not create a file to store the data." << endl <<
                                        "You will need to restart the sensor and try entering this state again." << endl;
                            }
                        } else if (data == 0) {
                            dataMeasurementStarted = false;
                            cout << "Data measurement not started successfully. This is a sensor-side error. Please try again." << endl;

                            // Once this has occurred enough times, the RPi should go back to sleep
                            if (failCommSensorCounter++ > MAX_COMM_FAIL_RETRIES)
                            {
                                state = SLEEP;
                                failCommSensorCounter = 0;
                            }
                             
                        }
                    } else {
                        dataMeasurementStarted = false;
                        cout << "Data measurement not started successfully. This is a communication error." << endl;
                        
                        // Once this has occurred enough times, the RPi should go back to sleep
                        if (failCommSensorCounter++ > MAX_COMM_FAIL_RETRIES)
                        {
                            
                            failCommSensorCounter = 0;

                            state = SLEEP;

                            cout << "Could not successfully establish communication with the mock sensor." << endl;
                            cout << "Putting RPi to sleep. Please make sure all wiring is correct." << endl << endl;
                        } else {
                            cout << "Attempt #" << failCommSensorCounter << " at attempting 'Init_Data_Measurement' command." << endl;
                        }
                    }
                } else {
                    cout << "Data measurement has already been started." << endl;
                    cout << "Initiating a Collect_Data command." << endl;
                    
                    // Once this has occurred enough times, the RPi should go back to sleep
                    if (failCommSensorCounter++ > MAX_COMM_FAIL_RETRIES)
                    {
                        failCommSensorCounter = 0;

                        state = SLEEP;

                        cout << "Continued entering back into 'Init_Data_Measure' state. Need to debug issue." << endl;
                        cout << "Putting RPi back to sleep." << endl << endl;
                    } else {
                        // Attempting to initiate 'Collect_Data' command again
                        state = COLLECT_DATA;
                    }
                }

                break;
            }
            // State S4
            case COLLECT_DATA:
            {
                cout << endl << "You are now in the Collect_Data state" << endl;
                cout << "Collected data: ";

                if (!dataTerminatedError)
                {
                    if (dataMeasurementStarted) {
                        // Initializing data collect buffer w/ a null byte to distinguish b/w when data
                        // is collected vs. not collected.
                        uint8_t data[COLLECT_DATA_MSG_SIZE] = {'\0'};

                        // Ensuring the RPi only samples the collected data at a rate of 2 Hz (500 ms period)
                        bcm2835_delay(DATA_SAMPLING_PERIOD);
                        if (sendCollectDataCommand(data)) {
                            if (data[0] != '\0') {
                                for (uint8_t byte = 0; byte < COLLECT_DATA_MSG_SIZE; byte++)
                                {
                                    if (byte < COLLECT_DATA_MSG_SIZE - 1) {
                                        cout << static_cast<int>(data[byte]) << ",";
                                    } else {
                                        cout << static_cast<int>(data[byte]) << endl;
                                    }
                                    // Only storing bytes in the next 'block' of the file if the limit has not been reached yet
                                    if (numBytesCollectAmount++ <= maxNumBytesToCollect) 
                                    {
                                        if (byte < COLLECT_DATA_MSG_SIZE - 1) {
                                            // Storing the byte in the file for later viewing
                                            *tempFile << static_cast<int>(data[byte]) << ",";
                                        } else {
                                            *tempFile << static_cast<int>(data[byte]) << endl;
                                        }
                                    } else {

                                        tempFile->close();
                                        numBytesCollectAmount = 0;

                                        // Terminating the data collection with the sensor
                                        if(sendTerminateDataCollectCommand())
                                        {
                                            dataMeasurementStarted = false;
                                            dataTerminatedError = false;
                                            state = DATA_PACKAGE;
                                        } else {
                                            // Ensuring the RPi is aware that it should try terminating the data collection process again
                                            dataTerminatedError = true;
                                        }
                                    
                                        cout << "Current data collection process has reached max num bytes. Moving onto packaging." << endl;
                                    }
                                }
                            } else {
                                cout << "The data is not ready to be collected yet. Please try again." << endl;

                                if (failCommSensorCounter++ > MAX_COMM_FAIL_RETRIES)
                                {
                                    // Resetting internal system status variables
                                    failCommSensorCounter = 0;

                                    dataMeasurementStarted = false;

                                    // Attempting to signal to RPi upon waking up that it needs to
                                    // try and correct the collect data error.
                                    dataTerminatedError = true;

                                    cout << "The mock sensor never got back into a state to collect data again." << endl;
                                    cout << "Please try waking the RPi back up to see if the mock sensor begins collecting data once more." << endl << endl;

                                    state = SLEEP;
                                } else {
                                    cout << "Attempt #" << failCommSensorCounter << " to re-try 'Data_Collect' command." << endl;
                                }
                            }
                        } else {
                            cout << "The sensor cannot be reached over I2C. Issuing the command again." << endl;
                            
                            if (failCommSensorCounter++ > MAX_COMM_FAIL_RETRIES)
                            {
                                // Resetting internal system status variables
                                failCommSensorCounter = 0;

                                dataMeasurementStarted = false;

                                cout << "Unable to re-establish connection with mock sensor over I2C. Please ensure pin connections are correct." << endl;
                                cout << "Putting the RPi back to sleep." << endl << endl;

                                state = SLEEP;
                            } else {
                                cout << "Attempt #" << failCommSensorCounter << " to re-try 'Data_Collect' command." << endl;
                            }
                            
                        }
                    } else {
                        cout << "The next Start_Measurement command has not been issued to the sensor yet." << endl;
                        cout << "You will need to successfully initiate the command before being able to try and collect the value." << endl;
                        state = INIT_DATA_MEASURE;
                    }
                } else {
                    // There was a data termination error reported, so continue issuing the terminating command
                    if(sendTerminateDataCollectCommand())
                    {
                        // There is no longer a termination error, so attempt to package and store and/or transmit the data
                        dataTerminatedError = false;
                        state = DATA_PACKAGE;
                    } else {
                        dataTerminatedError = true;
                    }
                }

                break;
            }
            // State S5
            case DATA_PACKAGE:
            {
                // Determining whether to save the data locally or move on to data transmission
                if (isConnectedToInternet())
                {
                    state = DATA_STORE;
                } else {
                    // The data must be saved locally
                    state = DATA_STORE;
                }

                break;
            }
            // State S6
            case TRANSMIT_DATA:
            {

                if (!transmitData(tempDataCollectFile_Name.str(), networkCreds::userName, networkCreds::password,
                                 networkCreds::serverIPAddr, networkCreds::fileTransferPath))
                {
                    // If the data transmission failed for some reason, proceed to storing it locally
                    state = DATA_STORE;
                }

                break;
            }
            // State S7
            case DATA_STORE:
            {
                stringstream localFileName = generateFileNameForDifferentFolder(tempDataCollectFile_Name.str(), dataCollectionFolderName);

                saveDataLocally(localFileName.str(), tempDataCollectFile_Name.str());

                dataCollectionNum++;    // Ensuring a new, unique temp file can be created on the next iteration

                // Moving back to sleep mode for now
                state = SLEEP;

                break;
            }
            default:
                break;
        }

        // For FABE: The OLED screen component was not able to be fully implemented.
        //           What is given below is the intended location for a call to update the OLED
        //           screen with the most up-to-date system information.
        // Updating the OLED screen with the most current status of the system
        // if (state != SLEEP)
        // {
        //     updateOledScreen(createShortenedStateName(state));
        //     bcm2835_delay(100);

        //     // After updating the OLED screen, switching back to the mock sensor to continue interaction
        //     bcm2835_i2c_setSlaveAddress(arduinoID);
        // }

        // Introducing general system delay to ensure state machine does not loop too quickly
        bcm2835_delay(100);
    }

    // Ensuring proper shutdown
    if (dataComm == I2C) {
        bcm2835_i2c_end();
        bcm2835_close();
    }

    return 0;
}

// State Handling Functions:

/**
 * Purpose: Send the I2C or Serial command to read the sensor device's status
 *          which is whether it is currently "on" or "off". Reports the
 *          feedback from the sensor.
 * Returns: True if command succeeded. False otherwise.
*/
bool sendReadDeviceStatusCommand() {
    // Setting the command and status
    char command = READ_DEVICE_STATUS_CMD;
    bool returnCodeRead = false;

    switch (dataComm) {
        case SERIAL:
        {
            // Writing the command until a response is received
            unsigned char serialReadBuf[READ_DEVICE_STATUS_MSG_SIZE] = {'\0'};  // Setting init val to use something to compare against
            while (RS232_PollComport(arduinoSerialPortNum, serialReadBuf, READ_DEVICE_STATUS_MSG_SIZE) <= 0) {
                RS232_SendByte(arduinoSerialPortNum, (unsigned char) command);
            }

            // Interpretting the return value from the sensor
            if (serialReadBuf[0] != '\0') {
                cout << "The sensor's status is: " << static_cast<int>(serialReadBuf[0]) << "." << endl;
                returnCodeRead = true;
            } else {
                cerr << "An error occurred when trying to read the response from the sensor over Serial." << endl;
            }
            break;
        }
        case I2C:
        {
            // Collecting and writing the command
            bcm2835_i2c_write(&command, 1);

            // Wait 35 ms before reading (allows for sensor processing time)
            bcm2835_delay(READ_DEVICE_STATUS_DELAY);

            // Read the result of the sensor query
            char i2cReadBuf[READ_DEVICE_STATUS_MSG_SIZE];
            uint8_t errCodeRead = bcm2835_i2c_read(i2cReadBuf, READ_DEVICE_STATUS_MSG_SIZE);

            returnCodeRead = interpretBCMErrCode(errCodeRead);
            if (returnCodeRead) {
                cout << "The sensor's status is: " << static_cast<int>(i2cReadBuf[0]) << "." << endl;
            } else {
                cerr << "An error occurred when trying to read the response from the sensor over I2C." << endl;
            }

            break;
        }
        default:    // Shouldn't be entered b/c default to I2C
            break;
    }

    return returnCodeRead;
}

/**
 * Purpose: Send the I2C or Serial "Begin_Data_Collection" command to the sensor to see if data is available.
 *          If data is available, then it will be read and stored into the 'data' variable.
 * Returns: True when data has been successfully read. False otherwise. Don't trust 'data' if the function returns false.
*/
bool sendBegDataCollectCommand(uint8_t & data) {
    // Setting the command and status
    char command = BEG_DATA_COLLECT_CMD;
    bool returnCodeRead = false;

    switch (dataComm) {
        case SERIAL:
        {
            // Writing the command until a response is received
            unsigned char serialReadBuf[BEG_DATA_COLLECT_MSG_SIZE] = {'\0'};  // Setting init val to use something to compare against
            while (RS232_PollComport(arduinoSerialPortNum, serialReadBuf, BEG_DATA_COLLECT_MSG_SIZE) <= 0) {
                RS232_SendByte(arduinoSerialPortNum, (unsigned char) command);
            }

            // Collecting the return value from the sensor if valid
            if (serialReadBuf[0] != '\0') {
                for (uint8_t i = 0; i < BEG_DATA_COLLECT_MSG_SIZE; i++) {
                    data = serialReadBuf[i];
                }
                returnCodeRead = true;
            }
            break;
        }
        case I2C:
        {
            // Issuing the command to the sensor
            bcm2835_i2c_write(&command, 1);

            // Wait 35 ms before reading (allows for sensor processing time)
            bcm2835_delay(BEG_DATA_COLLECT_DELAY);

            // Read the result of the sensor query
            char i2cReadBuf[BEG_DATA_COLLECT_MSG_SIZE];
            uint8_t errCodeRead = bcm2835_i2c_read(i2cReadBuf, BEG_DATA_COLLECT_MSG_SIZE);

            returnCodeRead = interpretBCMErrCode(errCodeRead);
            if (returnCodeRead) {
                for (uint8_t i = 0; i < BEG_DATA_COLLECT_MSG_SIZE; i++) {
                    data = i2cReadBuf[i];
                }
            }
            break;
        }
        default:    // Shouldn't be entered b/c default to I2C 
            break;
    }

    return returnCodeRead;
}

/**
 * Purpose: Send the I2C or Serial "Collect_Data" command to collect the next batch of data from the sensor.
 * Returns: True if command succeeded ('data' buffer now contains the transmitted data). False otherwise. Don't trust value of 'data' if false.
*/
bool sendCollectDataCommand(uint8_t * data) {
    // Setting the command and status
    char command = COLLECT_DATA_CMD;
    bool returnCodeRead = false;

    switch (dataComm) {
        case SERIAL:
        {
            // Writing the command until a response is received
            unsigned char serialReadBuf[COLLECT_DATA_MSG_SIZE] = {'\0'};  // Setting init val to use something to compare against
            while (RS232_PollComport(arduinoSerialPortNum, serialReadBuf, COLLECT_DATA_MSG_SIZE) <= 0) {
                RS232_SendByte(arduinoSerialPortNum, (unsigned char) command);
            }

            // Collecting the return value from the sensor if valid
            if (serialReadBuf[0] != '\0') {
                memcpy(data, serialReadBuf, COLLECT_DATA_MSG_SIZE);
                returnCodeRead = true;
            }
            break;
        }
        case I2C:
        {
            // Issuing the command to the sensor
            bcm2835_i2c_write(&command, 1);

            // Wait 35 ms before reading (allows for sensor processing time)
            bcm2835_delay(COLLECT_DATA_DELAY);

            // Read the result of the sensor query - expect 32 bytes
            char i2cReadBuf[COLLECT_DATA_MSG_SIZE];

            uint8_t errCodeRead = bcm2835_i2c_read(i2cReadBuf, COLLECT_DATA_MSG_SIZE);

            returnCodeRead = interpretBCMErrCode(errCodeRead);

            if (returnCodeRead) {
                memcpy(data, i2cReadBuf, COLLECT_DATA_MSG_SIZE);
            }
        }
        default: // Shouldn't be entered b/c default to I2C
            break;
    }

    return returnCodeRead;
}

/**
 * Purpose: Send the I2C or Serial "Terminate Data Collection" command to terminate the current data collection process.
 * Returns: True if command succeeded, false otherwise.
*/
bool sendTerminateDataCollectCommand() {
    // Setting the command and status
    char command = TERMINATE_DATA_COLLECT_CMD;
    bool returnCodeRead = false;

    switch (dataComm) {
        case SERIAL:
        {
            // Writing the command until a response is received
            unsigned char serialReadBuf[TERM_DATA_COLLECT_MSG_SIZE] = {'\0'};  // Setting init val to use something to compare against
            while (RS232_PollComport(arduinoSerialPortNum, serialReadBuf, TERM_DATA_COLLECT_MSG_SIZE) <= 0) {
                RS232_SendByte(arduinoSerialPortNum, (unsigned char) command);
            }

            // Collecting the return value from the sensor if valid
            if (serialReadBuf[0] != '\0') {
                
                // The sensor reported that it terminated successfully
                if (static_cast<int>(serialReadBuf[0]) == 1)
                {
                    returnCodeRead = true;
                } else {
                    returnCodeRead = false;
                }
            }
            break;
        }
        case I2C:
        {
            // Issuing the command to the sensor
            bcm2835_i2c_write(&command, 1);

            // Wait 35 ms before reading (allows for sensor processing time)
            bcm2835_delay(TERM_DATA_COLLECT_DELAY);

            // Read the result of the sensor query - expect 32 bytes
            char i2cReadBuf[TERM_DATA_COLLECT_MSG_SIZE];

            uint8_t errCodeRead = bcm2835_i2c_read(i2cReadBuf, TERM_DATA_COLLECT_MSG_SIZE);

            returnCodeRead = interpretBCMErrCode(errCodeRead);

            if (returnCodeRead) {
                
                // The sensor reported that it terminated successfully
                if (static_cast<int>(i2cReadBuf[0]) == 1)
                {
                    returnCodeRead = true;
                } else {
                    returnCodeRead = false;
                }
            }
        }
        default: // Shouldn't be entered b/c default to I2C
            break;
    }

    return returnCodeRead;
}

/**
 * Purpose: Save an already existing temporary file into a permanent location @ <newFileName>
 * Returns: True if all data was successfully copied into the permanent location @ <newFileName>.
 *          False otherwise.
*/
bool saveDataLocally(const string& newFileName, const string& tempFileName) {
    
    bool operationStatus = false;
    
    // Opening the input and output file streams
    ifstream tempFile(tempFileName);
    ofstream newFile(newFileName);

    // Copying all contents from the temp file to the new file
    newFile << tempFile.rdbuf();

    // Ensuring all data is located within the new file
    tempFile.seekg(0, ios::end);
    int tempFileSize = tempFile.tellg();
    newFile.seekp(0, ios::end);
    int newFileSize = newFile.tellp();

    if (newFileSize == tempFileSize)
    {
        // All data was successfully stored into the new file
        operationStatus = true;
    }

    return operationStatus;
}

// Wireless Communication Function Definitions:

bool isConnectedToInternet() {
    // Attempt to ping Google's public DNS server to check for internet connectivity
    bool status = system("ping -c 1 8.8.8.8 > /dev/null 2>&1") == 0;
    if (status) {
        cout << "System network connection successful" << endl;
    }
    return status;
}

bool transmitData(const string& fileName, const string& userName, const string& password, const string& ipAddress, const string& fileDestination)
{
    bool operationStatus = false;

    string command = "scp " + fileName + " " + userName + "@" + ipAddress + ":/" + fileDestination;

    if (system(command.c_str()) == 0)
    {
        cout << "Data successfully transmitted to " << ipAddress << endl;

        // Attempting to remove the locally stored temp file used in data transmission
        if (remove(fileName.c_str()))
        {
            cout << "Local copy deleted: " << fileName << endl;
            operationStatus = true;
        }
    }

    return operationStatus;
}


// Utility Functions:

/***
 * Purpose: Called when the Ctr-C keys is pressed to safely exit the program
*/
void sigIntHandler(int signum)
{
    stopProgram = 1;
}

/**
 * Purpose: Interpret the passed in bcm2835 error code to determine if
 *          the error requires adjustment of the current program operation or not.
 * Returns: True if the error code does not require adjustment of the current program or
 *          False if it does.
*/
bool interpretBCMErrCode(uint8_t code) {
    bool returnCode = false;
    switch (code) {
        case BCM2835_I2C_REASON_OK:
            returnCode = true;
            break;
        case BCM2835_I2C_REASON_ERROR_NACK:
        case BCM2835_I2C_REASON_ERROR_CLKT:
        case BCM2835_I2C_REASON_ERROR_DATA:
        case BCM2835_I2C_REASON_ERROR_TIMEOUT:
            break;
    }

    return returnCode;
}

/**
 * Purpose: Generate the next available file name to represent the
 *          next data collection cycle. This will be stored within the corresponding folder. 
 * Returns: A string representing the most up-to-date name which is of the following form:
 *          "<folderName>/%Y%m%d_%H%M%S_<fileNum>"
*/
stringstream generateNextFileName(const string& folderName, int fileNum) {

    // Obtaining the most recent time from the system
    auto now = chrono::system_clock::now();
    auto now_c = chrono::system_clock::to_time_t(now);

    // Generating the new file name string
    stringstream fileName;
    fileName << folderName << "/" << put_time(localtime(&now_c), "%Y%m%d_%H%M%S") << "_" << to_string(fileNum) << ".txt";

    return fileName;
}

/**
 * Purpose: Takes @ <prevFileName> and adjusts it to exist in the <newFolderName>.
 *          Expects <prevFileName> to be of the form: "<prevFolderName>/...." or else it
 *          will not convert the file name correctly.
 * Returns: The converted file name of the form: <newFolderName>/<prevFileName w/out original folder>
*/
stringstream generateFileNameForDifferentFolder(const string & prevFileName, const string & newFolderName)
{
    // Finding the position of the first occurrence of the "/" character
    int forSlashCharPos = prevFileName.find("/");

    // Extracting the remainder of the string from the "/" character to the end and prepending the new folder name
    stringstream newFileName;
    newFileName << newFolderName << prevFileName.substr(forSlashCharPos);

    return newFileName;
}

/**
 * Purpose: Create a shortened version of the current <state>'s name.
 *          This can be used to summarize where in general the RPi is
 *          at in its operation cycle.
 * Returns: A string containing the shortened state name.
*/
std::string createShortenedStateName(mockSensorStates state)
{
    std::string shortenedStateName;

    switch (state)
    {
        case SLEEP:
            shortenedStateName = "Sleep";
            break;
        case CHECK_SENSOR_ON:
            shortenedStateName = "On";
            break;
        case WAIT_FOR_RETRY_CHECK:
            shortenedStateName = "On";
            break;
        case INIT_DATA_MEASURE:
            shortenedStateName = "On";
            break;
        case COLLECT_DATA:
            shortenedStateName = "Data Collect";
            break;
        case DATA_PACKAGE:
            shortenedStateName = "Data Collect";
            break;
        case TRANSMIT_DATA:
            shortenedStateName = "Data Transmit";
            break;
        case DATA_STORE:
            shortenedStateName = "Data Store";
            break;
        default:
            shortenedStateName = "Unknown";
            break;
    }

    return shortenedStateName;
}
