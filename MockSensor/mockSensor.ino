#include <Wire.h>

#define DEBUG // Ensure this is commented out if Arduino is connected directly to the RPi
#define MESSAGE_BUF_SIZE 1
#define POLL_COUNT_MAX 2          // This defines the maximum amount of polls per data collection period that the RPi will need to provide
#define DEVICE_BUTTON_PIN 8       // This defines the pin on the sensor that is used to simulate the device's "on" and "off" states
#define TOTAL_COMMAND_COUNT 4     // This defines the total number of commands that the sensor can accept over I2C
#define DATA_BUF_SIZE 32          // This defines the total number of bytes that are collected by the sensor - the max value supported in the buffer is 32 bytes
#define SWITCH_UPPER_THRESH 900
#define SWITCH_LOWER_THRESH 100

// Defining important pin connection values
const uint8_t randomValAnalogPin = A0;
const uint8_t switchAnalogPin = A1;
const uint8_t serialCommsIndicator = 4;
const uint8_t i2cCommsIndicator = 2;
const uint8_t noCommsIndicator = 3;

// Defining important communication type parameters
int commType;                           // Used to indicate what type of communication is being used for this sensor

// Defining important i2c communication parameters
uint8_t i2c_ID = 2;
uint8_t i2c_RPi_ID = 4;
volatile bool msgReceived = false;      // Used to indicate whether a message has been received from the RPi
                                        // Also is used to control the entry into any of the possible states of the sensor

// Defining important simulation parameters
uint8_t deviceStatus;                     // Used to indicate whether the device is "on" or "off"
uint8_t dataStatus;                       // Used to indicate whether the data is ready to be collected or not - will be "0" if data is not ready yet
uint8_t dataCollectionStatus;           // Used to indicate whether the "Collect_Data" command was executed successfully
uint8_t dataTerminateStatus = 0;              // Used to indicate whether the "Terminate Data Collection" command was executed successfully - 1 for success, 0 for failure    

bool dataCollectionInitiated = false;     // Used to indicate whether a data collection command has been received already. Used to prevent multiple simultaneous data collection processes.


// Defining state names as an enum
volatile enum mockSensorStates {REPORT_DEVICE_STATUS, BEGIN_DATA_COLLECTION, COLLECT_DATA, TERMINATE_DATA_COLLECTION} state;

// Defining possible command's that can be interpreted by the sensor
/**
 * The commands are as follows:
 *    - 0x01 --> Read_Device_Status
 *    - 0x02 --> Begin_Data_Collection
 *    - 0x03 --> Collect_Data
 *    - 0x04 --> Terminate_Data_Collection
 */                        
constexpr uint8_t commands[TOTAL_COMMAND_COUNT] = { 0x01, 0x02, 0x03, 0x04 }; 

// Defining 1 byte data buffer - all messages should be interpreted as a possible sensor state for the Arduino to be in
uint8_t msgBuf[MESSAGE_BUF_SIZE];

// Defining a 1 byte data buffer representing the data collected by the sensor device
volatile uint8_t collectedDataBuf[DATA_BUF_SIZE];

// Function prototypes for setting the messaging format of the sensor
int setMessageType();

// Function protoypes for I2C message callbacks:
void handleReceivedMessage();
void handleRequestMessage();

// Function prototypes for handling device states
uint8_t handleReportDeviceStatus();
void handleCollectData();
void handleReportDataNotReadyStatus();
void handleReportDataReadyStatus();

// Function prototype for decoding received message
bool decodeMsgToState(uint8_t * msg);

// TODO: 
// Main Operations:
void setup() {

  Serial.begin(9600); // Used for debugging purposes

  #ifdef DEBUG
  Serial.println(F("Beginning fakeSensor_Version2.ino"));
  #endif

  // Relying on the floating nature of the pin when set to input to properly seed the random val generator.
  randomSeed(analogRead(randomValAnalogPin));

  // Initializing pins required for functionality of onboard 'switch' to set Serial or I2C comms
  pinMode(switchAnalogPin, INPUT);
  pinMode(serialCommsIndicator, OUTPUT);
  pinMode(i2cCommsIndicator, OUTPUT);
  pinMode(noCommsIndicator, OUTPUT);
  
  // Initializing a digital pin for simulating the process of turning the sensor "on" and "off"
  pinMode(DEVICE_BUTTON_PIN, INPUT_PULLUP);

  // Initializing the device's state of being on or off
  deviceStatus = handleReportDeviceStatus();

  // Reading the onboard switch to determine whether to communicate in Serial or I2C mode
  // Ensuring that a valid comms type is selected prior to proceeding (0 indicates invalid)
  commType = 0;
  while (commType == 0) {
    commType = setMessageType();
    // Indicating that the switch was not set in a valid state
    if (commType == 0) {
      digitalWrite(noCommsIndicator, HIGH);
    } else {
      digitalWrite(noCommsIndicator, LOW);
    }
  }

  Serial.print("The selected comm type is: ");
  Serial.println(commType);

  switch (commType){
    // Indicating I2C Comms were selected
    case 2:
      digitalWrite(i2cCommsIndicator, HIGH);

      // Initializing the I2C interface to make the Arduino
      // the client in the communication
      Wire.begin(i2c_ID);

      // Defining the callback function when the Arduino receives a message
      // from the RPi
      Wire.onReceive(handleReceivedMessage);

      // Defining the callback function when the Arduino receives a read request
      // from the RPi
      Wire.onRequest(handleRequestMessage);

      break;
    // Indicating Serial Comms were selected
    case 1:
      digitalWrite(i2cCommsIndicator, HIGH);
      break;
  }
}

void loop() {
  // Adjusting the sensor's ability to wait for a command if Serial comms was selected
  if (commType == 1) {
    // Only collecting and decoding the message command if there is something available in the Serial buffer
    while (Serial.available() <= 0);
    Serial.readBytes(msgBuf, MESSAGE_BUF_SIZE);

    // Decode captured message into a possible state
    if (decodeMsgToState(msgBuf))
    { 
      // Ensuring it is known that a message was received for proper message reading
      msgReceived = true;
    } else {
      msgReceived = false;
    }
  }
  
  // Only perform a state action when a new command has been read over I2C from the RPi
  if (msgReceived)
  {
    // Scanning for possible codes sent by RPi to indicate
    // the sensor to emulate another response
    switch(state) {
      // Collecting information about whether or not the device is "on"
      case REPORT_DEVICE_STATUS:
        deviceStatus = handleReportDeviceStatus();
        break;

      case BEGIN_DATA_COLLECTION:
        // Ensuring data collection has not already been initiated
        if (!dataCollectionInitiated)
        {
            dataCollectionInitiated = true;

            // Ensuring the RPi is aware that data collection was started successfully
            dataStatus = 1;

            #ifdef DEBUG
              Serial.println("Handling \"Begin Data Collection\" state");
              Serial.println("Sent a value of 1 to the RPi to indicate data collection was initiated!");
            #endif
        } else {
            dataStatus = 0;
        }
        break;

      // State that is entered when the RPi requests to collect data.
      case COLLECT_DATA:
        // Ensuring the collect data state is not entered more than once an entire polling cycle - this would overwrite the collected data
        if (dataCollectionInitiated) {

          // Collecting the data
          handleCollectData();

          dataCollectionStatus = 1;

          #ifdef DEBUG
          Serial.println("Handling \"Collect Data\" state");
          #endif
        } else {
          dataCollectionStatus = 0;
          #ifdef DEBUG
          Serial.println("Did not actually Collect Data.");
          #endif
        }
        break;
      
      // State that is entered when the RPi is finished collecting data from the device
      case TERMINATE_DATA_COLLECTION:

        #ifdef DEBUG
        Serial.println(F("Handling \"Terminate Data Collection\" state"));
        #endif
        // Ensuring the RPi can begin collecting from the sensor at a different time
        dataCollectionInitiated = false;
        dataTerminateStatus = 1;

        break;
 
      default:
        #ifdef DEBUG
        Serial.print(F("The following state code received is not currently registered: "));
        Serial.println(state);
        #endif
        break;
    }

    // Ensuring a state is not repeatedly entered. This requires the RPi to initiate queries.
    msgReceived = false;
    // #ifdef DEBUG
    // Serial.print(F("The current state is = "));
    // Serial.println(state);
    // #endif
    // delay(10); // Delaying for 10 miliseconds
  }
  #ifdef DEBUG_ANNOYING
  Serial.println(F("No message has been received yet!"));
  #endif
}

// Function definitions

// Purpose: Check the onboard switch to determine whether to use I2C or Serial for comms
// Returns: 2 if I2C comms was selected, 1 if Serial comms was selected, and 0 if the value cannot be determined
int setMessageType() {
  int returnStatus;
  int switchVal = analogRead(switchAnalogPin);

  // #ifdef DEBUG
  // Serial.println(switchVal);
  // #endif

  if (switchVal > SWITCH_UPPER_THRESH)
  {
    // The user has indicated they have selected I2C comms - the knob has been turned almost all the way ccw
    returnStatus = 2;
  } else if (switchVal < SWITCH_LOWER_THRESH) {
    // The user has indicated they have selected Serial comms - the knob has been turned almost all the way cw
    returnStatus = 2;
  } else {
    // The comm type cannot be determined
    returnStatus = 0;
  }
  
  return returnStatus;
}


// Purpose: (For I2C ONLY) Decode the received message into a command that will be interpreted by the sensor
void handleReceivedMessage()
{
  uint8_t buffIdx = 0;
  // Ensuring data is truly available
  while (Wire.available() && buffIdx < MESSAGE_BUF_SIZE)
  {
    msgBuf[buffIdx] = Wire.read();
    buffIdx++;
  }

  // Decode captured message into a possible state
  if (decodeMsgToState(msgBuf))
  { 
    // Ensuring it is known that a message was received for proper message reading
    msgReceived = true;
  } else {
    msgReceived = false;
  }
}

// Purpose: (For I2C ONLY) Send the desired data back to the RPi when it initiates a read request
void handleRequestMessage() {
  // Ensuring the proper data is transmitted based on the current state
  switch(state) {
    case REPORT_DEVICE_STATUS:
      Wire.write(deviceStatus);
      break;
    // Only will be sending data status when the data isn't ready to be collected
    case BEGIN_DATA_COLLECTION:
      Wire.write(dataStatus);
      break;
    // Letting the RPi know that the status of the "Start_Measurement" command
    case COLLECT_DATA:
      Wire.write(dataCollectionStatus);
      if (dataCollectionStatus == 1) {
        const char *collectedDataToSend = collectedDataBuf;
        Wire.write(collectedDataToSend);
      }
      break;
    case TERMINATE_DATA_COLLECTION:
      Wire.write(dataTerminateStatus);
      break;
  }
}

// Purpose: Reads a toggle button to simulate whether or not the sensor device is on
// Returns: A "0" for when the device is registered to be "off" and a "1" when the device is registered to be "on" (default)
uint8_t handleReportDeviceStatus() {
    #ifdef DEBUG
      Serial.println(F("Entered into Sensor Device Pre-Check"));
    #endif

    uint8_t status;
    if (digitalRead(DEVICE_BUTTON_PIN) == LOW) {
      // The device is "off"
      status = 0;        
    } else {
      // The device is "on" - default
      status = 1;    
    }

    // Sending the value out right away if Serial comms was set
    if (commType == 1)
    {
      Serial.write(status);
    }

    #ifdef DEBUG
    Serial.print(F("The device status is = "));
    Serial.println(status);
    #endif

    return status;
}

void handleCollectData()
{
  // Collecting data to be sent and filling message buffer
  #ifdef DEBUG
  Serial.print("Collected the following data: ");
  #endif

  for (uint8_t i = 0; i < DATA_BUF_SIZE; i++) {
      uint8_t data;
 
      if (i < DATA_BUF_SIZE - 1)
      {
        // Generating a random "sensor" reading point from 0-255
        data = random(256);
        collectedDataBuf[i] = data;
      } else {
        // Ensuring the string is null-terminated to allow for proper interpretation
        collectedDataBuf[i] = '\0';
      }

      #ifdef DEBUG
      Serial.print(data);
      if (i < DATA_BUF_SIZE - 1) {
        Serial.print(", ");
      } else {
        Serial.println();
      }
      #endif
  }

  // Transmitting the collected data buffer out over Serial if that comm type was selected
  if (commType == 1) {
    if (Serial.availableForWrite() < DATA_BUF_SIZE) {
      // Ensuring all previous data is sent out of the Serial buffer
      Serial.flush();
    } else {
      // 32 bytes are at least available on the buffer to write, so send them out
      const char *collectedDataToSend = collectedDataBuf;
      Serial.write(collectedDataToSend, DATA_BUF_SIZE);
    }
  }
}


// Purpose: Decodes the message within the message buffer into a possible state that defines what the sensor will do.
// Requirement: msg buffer cannot be null
// Returns: A boolean value corresponding to whether or not the message was successfully decoded (true for decoded message
//          and false for decode failure). If false, then it is not recommended to use the received message from the RPi.
bool decodeMsgToState(uint8_t * msg) {

  bool successfulDecode = true;
  // Decoding the byte stored in the buffer and setting the internal state based on the byte value
  switch(msg[0]) {
    // Decoding the Read_Device_Status command from the RPi
    case commands[0]:
      state = REPORT_DEVICE_STATUS;
      break;

    // Decoding the Read_Data_Ready_Flag command from the RPi
    case commands[1]:
      state = BEGIN_DATA_COLLECTION;
      break;

    // Decoding the Start_Measurement command from the RPi
    case commands[2]:
      state = COLLECT_DATA;
      break;
    
    // Decoding the Terminate_Data_Collection command from the RPi
    case commands[3]:
      state = TERMINATE_DATA_COLLECTION;
      break;
    
    default:
      // The command in the message buffer does not correspond to a valid state in the sensor
      successfulDecode = false;
      #ifdef DEBUG
        Serial.println(F("The command could not be decoded!"));
      #endif
      break;
  }

  // #ifdef DEBUG
  // Serial.print(F("The decoded state is = "));
  // Serial.println(state);
  // #endif

  // Ensuring all values in the I2C message buffer are set to -1.
  // This ensures that the message relay can be properly debugged as necessary.
  memset(msgBuf, -1, MESSAGE_BUF_SIZE);

  return successfulDecode;
}