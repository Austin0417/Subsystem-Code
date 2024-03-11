#include "BLEDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <string.h>
#include <map>
#include <functional>
#include <future>

// BLEService and BLECharacteristic UUID
// These should match the service UUID from the main system, and the characteristic UUID of the subsystem's characteristic on the main system
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SUBSYSTEM_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"

// Macros for the pin numbers for the ultrasonic sensors and LEDs
#define FIRST_TRIG_PIN 2
#define FIRST_ECHO_PIN 15
#define SECOND_TRIG_PIN 25
#define SECOND_ECHO_PIN 33
#define CONNECTION_LED_PIN 13
#define STATUS_LED_PIN 0

// This is responsible for determining the length of the "hold" of the ultrasonics. Basically how long it stays active after it detected someone walking by
#define PROXIMITY_PAUSE 3000

#define ULTRASONIC_ONE 1
#define ULTRASONIC_TWO 2

#define NUM_SYSTEMS 1

BLEScan* scanner;
BLEAdvertisedDevice targetDevice;

// BLECharacteristic that we will be writing to
BLERemoteCharacteristic* characteristic;

bool shouldAttemptConnect = false;
// If connection to the main ESP32 succeeds, this will be set to true
bool shouldStart = false;


void sendBluetoothMessage(String message, BLERemoteCharacteristic* characteristic) {
  uint8_t data[message.length() + 1];
  memcpy(data, message.c_str(), message.length());
  characteristic->writeValue(data, message.length());
}

////////////////////////////////////////////////////////////////////////////////////////
// Helper class to manage the current room occupation status
// Will use singleton design pattern
////////////////////////////////////////////////////////////////////////////////////////

class RoomOccupationStatus {
  private:
    static RoomOccupationStatus* instance;
    bool occupied = false;
    String roomStatus = "Not occupied"; // roomStatus will be indirectly set through setOccupied and setUnoccupied
    unsigned long time_since_last_status_set_ = 0;
  public:
    static RoomOccupationStatus* getInstance();
    bool isOccupied() const;
    String getRoomStatus() const;
    void setOccupied();
    void setUnoccupied();
};

RoomOccupationStatus* RoomOccupationStatus::instance = nullptr;

RoomOccupationStatus* RoomOccupationStatus::getInstance() {
  if (instance == nullptr) {
      instance = new RoomOccupationStatus;
  }
  return instance;
}

bool RoomOccupationStatus::isOccupied() const { return occupied; }
String RoomOccupationStatus::getRoomStatus() const { return roomStatus; }

void RoomOccupationStatus::setOccupied() { 
  if (millis() - time_since_last_status_set_ < 1500) {
    return;
  }
  occupied = true;
  roomStatus = "Occupied";
  time_since_last_status_set_ = millis();
}

void RoomOccupationStatus::setUnoccupied() {
  if (millis() - time_since_last_status_set_ < 1500) {
    return;
  }
  occupied = false;
  roomStatus = "Not occupied";
  time_since_last_status_set_ = millis();
}

// Singleton instance of RoomOccupationStatus
RoomOccupationStatus* roomOccupationInstance = RoomOccupationStatus::getInstance();

////////////////////////////////////////////////////////////////////////////////////////
// Helper class to manage subsystem's connections to potentially multiple main systems
// Uses Singleton design pattern as well
// This isn't really that important since currently, there is only 1 main system in use
////////////////////////////////////////////////////////////////////////////////////////
class ConnectionsManager {
  private:
  // Storing references to remote characteristics of all systems within a room
  std::map<BLEClient*, BLERemoteCharacteristic*> clientCharacteristicMapping;   // map which links every client connection (to each main system) to the respective characteristics on that main system
  std::map<BLEAddress, bool> addressMapping;  // map to keep track of each main system (key is unique bluetooth address, value is boolean representing current connection to the device)
  static ConnectionsManager* instance;
  public:
  static ConnectionsManager* getInstance();
  void addClientCharacteristicPair(BLEClient* client, BLERemoteCharacteristic* characteristic);
  bool removeClientCharacteristicPair(BLEClient* client);
  void setAddressMapping(BLEAddress deviceAddress, bool connectionStatus);
  bool isConnectedToDevice(BLEAddress address);
  void setValuesForCharacteristics(String message);
  static void sendValues(ConnectionsManager* connectionManager);
  bool shouldStart() const;
};

ConnectionsManager* ConnectionsManager::instance = nullptr;

ConnectionsManager* ConnectionsManager::getInstance() {
  if (instance == nullptr || instance == NULL) {
    instance = new ConnectionsManager;
  }
  return instance;
}

void ConnectionsManager::sendValues(ConnectionsManager* connectionManager) {
  connectionManager->setValuesForCharacteristics(roomOccupationInstance->getRoomStatus());
}

void ConnectionsManager::addClientCharacteristicPair(BLEClient* client, BLERemoteCharacteristic* characteristic) {
  clientCharacteristicMapping[client] = characteristic;
}

bool ConnectionsManager::removeClientCharacteristicPair(BLEClient* client) {
   return clientCharacteristicMapping.erase(client) > 0;
}

void ConnectionsManager::setAddressMapping(BLEAddress address, bool connectionStatus) {
  addressMapping[address] = connectionStatus;
}

bool ConnectionsManager::isConnectedToDevice(BLEAddress address) {
  if (addressMapping.find(address) == addressMapping.end() || !addressMapping[address]) {
    return false;
  } else {
    return true;
  }
}

// Method to check if the subsystem is connected successfully to all the ESP32 systems
bool ConnectionsManager::shouldStart() const {
  if (addressMapping.empty()) {
    return false;
  }
  for (auto it = addressMapping.begin(); it != addressMapping.end(); ++it) {
    if (!(it->second)) {
      return false;
    }
  }
  return true;
}

void ConnectionsManager::setValuesForCharacteristics(String message) {
  for (auto it = clientCharacteristicMapping.begin(); it != clientCharacteristicMapping.end(); ++it) {
    sendBluetoothMessage(message, it->second);
  }
}

// Singleton ConnectionsManager object that will manage a list of all of the characteristics of the systems in the room
ConnectionsManager* connectionsManager = ConnectionsManager::getInstance();

////////////////////////////////////////////////////////////////////////////////////////
// Helper class that will send the room occupation status to all main systems asynchronously
////////////////////////////////////////////////////////////////////////////////////////
class BluetoothMessenger {
  private:
  static BluetoothMessenger* instance;
  static bool shouldSendStatus;
  public:
  static BluetoothMessenger* getInstance();
  void initializeMessageService();
  void startMessageService();
  void stopMessageService();
};

BluetoothMessenger* BluetoothMessenger::instance = nullptr;
bool BluetoothMessenger::shouldSendStatus = false;

BluetoothMessenger* BluetoothMessenger::getInstance() {
  if (instance == nullptr) {
    instance = new BluetoothMessenger;
  }
  return instance;
}

void BluetoothMessenger::initializeMessageService() {
  xTaskCreatePinnedToCore([] (void* args) {
    Serial.println("Initializing asynchronous BLE messenger...");
    while (true) {
      if (shouldSendStatus) {
        connectionsManager->setValuesForCharacteristics(roomOccupationInstance->getRoomStatus());
      }
    }
    }, "BLUETOOTH_MESSENGER", 10000, NULL, 0, NULL, 0);
}

void BluetoothMessenger::startMessageService() { shouldSendStatus = true; }
void BluetoothMessenger::stopMessageService() { shouldSendStatus = false; }

////////////////////////////////////////////////////////////////////////////////////////
// Class inheriting from BLEClientCallbacks which implements pure virtual methods (onConnect and onDisconnect) which serve as callbacks
////////////////////////////////////////////////////////////////////////////////////////
class MyClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* client) override;
    void onDisconnect(BLEClient* client) override;
};

void MyClientCallbacks::onConnect(BLEClient* client) {
  
}

// If the subsystem has disconnected from the main system, start attempting to reconnect by setting the shouldStart flag back to false
void MyClientCallbacks::onDisconnect(BLEClient* client) {
  if (connectionsManager->removeClientCharacteristicPair(client)) {
    Serial.print("Disconnected from ");
    Serial.print(client->toString().c_str());
    Serial.print(" with peer address: ");
    Serial.println(client->getPeerAddress().toString().c_str());
  }
  connectionsManager->setAddressMapping(client->getPeerAddress(), false);
  shouldStart = false;
}

// Function that connects to a given advertised device
// This is used when the scan picks up the main system
// Returns true if the connect was successful, false otherwise
bool connectToServer(BLEAdvertisedDevice* device) {
  Serial.println("Beginning BLE connection with main ESP32...");
  // Obtain the client handle
  BLEClient* client = BLEDevice::createClient();
  if (client == nullptr) {
    Serial.print("Error establishing BLEClient");
    return false;
  } else {
    Serial.println("BLEClient success");
  }
  client->setClientCallbacks(new MyClientCallbacks());
  client->connect(device);

  // Attempt a connection to the BLEService with the target service UUID
  BLERemoteService* remoteService = client->getService(SERVICE_UUID);
  if (remoteService == nullptr) {
    Serial.println("Error connecting to remote service");
    return false;
  } else {
    Serial.println("Remote Service success!");
  }
  Serial.print("Connected to service with UUID: ");
  BLEUUID serviceUUID = remoteService->getUUID();
  std::string serviceUUIDString = serviceUUID.toString();
  Serial.println(serviceUUIDString.c_str());

  // Find the target characteristic within the service after a successful connection to the service
  characteristic = remoteService->getCharacteristic(SUBSYSTEM_CHARACTERISTIC_UUID);
  if (characteristic == nullptr) {
    Serial.println("Error connecting to target characteristic");
    return false;
  } else {
    Serial.println("Remote characteristic success!");
  }
  connectionsManager->addClientCharacteristicPair(client, characteristic);
  std::string characteristicString = characteristic->toString();
  Serial.print("Connected to characteristic: ");
  Serial.println(characteristicString.c_str());
  return true;
}


// Class that implements the BLEAdvertisedDeviceCallbacks interface
class AdvertisedDeviceCallback : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override;
};

// On a scan result, we check to scanned device's name to see if it matches the main system (in this case the main system is just called "ESP32")
void AdvertisedDeviceCallback::onResult(BLEAdvertisedDevice advertisedDevice) {
  Serial.println(advertisedDevice.getName().c_str());
  // When the scanner picks up an ESP32 and the subsystem is not yet connected to it, attempt to connect
  // Set targetAddress and the shouldAttemptConnect flag
  if (advertisedDevice.getName() == std::string("ESP32") && !connectionsManager->isConnectedToDevice(advertisedDevice.getAddress())) {
    Serial.println("Located main ESP32");
    BLEAddress targetAddress = advertisedDevice.getAddress();
    std::string addressString = targetAddress.toString();
    Serial.print("ESP32 Address: ");
    Serial.println(addressString.c_str());
    targetDevice = advertisedDevice;
    shouldAttemptConnect = true;
  }
}


// Function that will start scanning nearby BLE devices
void startScan() {
  Serial.println("Starting BLE scan...");

  // Start scanning for 15 seconds
  // In this time, AdvertisedDeviceCallback::onResult will be called every time a device is picked up
  scanner->start(15);

  // shouldAttemptConnect will be true if the scan picked up the main ESP32 system
  if (shouldAttemptConnect) {
    shouldStart = connectToServer(&targetDevice);
    connectionsManager->setAddressMapping(targetDevice.getAddress(), shouldStart);
    if (shouldStart) {
      // Turn the LED on to indicate connection was successful
      digitalWrite(CONNECTION_LED_PIN, HIGH);
    }
  }
}

void clearClientInfo() {
  shouldAttemptConnect = false;
  
}

// This function will be called when the subsystem needs to reconnect to the main system
// The subsystem will try every 30 seconds to reconnect by scanning. If the main system isn't picked up, it will try again in 30 seconds
void retryConnection() {
  clearClientInfo();
  Serial.println("Couldn't find/connect to the host ESP32 server. Reattempting connection in 30s.");
  for (int i = 0; i < 15; i++) {
    Serial.print(".");
    digitalWrite(CONNECTION_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(CONNECTION_LED_PIN, LOW);
    delay(1000);
  }
  startScan();
}


////////////////////////////////////////////////////////////////////////////////////////
// Proximity sensor class definition/parameters
// Trig pin goes to GIOP 2
// Echo pin goes to GIOP 15
////////////////////////////////////////////////////////////////////////////////////////
class UltrasonicSensor {
  private:
    int trigger_pin;
    int echo_pin;
    long duration;
    int distance_;
    int calibratedValue = 0;
    bool isCalibrated = false;
    long unsigned int lastActiveProximity = 0;
    int sensorNumber;

    // Boolean that indicates status of the ultrasonic sensor (active or nonactive)
    bool detected = false;
    bool previousStatus = false;

    // Flag that is set to true whenenver the vibration sensor is active
    // When the system is moved, we need to recalibrate the ultrasonic's baseline
    bool shouldRetakeBaseline = false;

  public:
    UltrasonicSensor(int trig_pin, int echo_pin, int sensor_number) {
      pinMode(trig_pin, OUTPUT);
      pinMode(echo_pin, INPUT);
      trigger_pin = trig_pin;
      this->echo_pin = echo_pin;
      sensorNumber = sensor_number;
    }

    int distance() {
      return distance_;
    }

    bool isActive() {
      return detected;
    }

    void shouldRecalibrate() {
      shouldRetakeBaseline = true;
    }

    long unsigned int getDetectionTime() {
      return lastActiveProximity;
    }

    int baseline() {
      return calibratedValue;
    }

    int calculateDistance() {
      digitalWrite(trigger_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin, LOW);

      duration = pulseIn(echo_pin, HIGH);

      distance_ = duration * 0.034 / 2;
      return distance_;
    }

    void compareCurrentWithBaseline() {
      if (!isCalibrated || shouldRetakeBaseline) {
        calibratedValue = distance_;
        isCalibrated = true;
        Serial.print("Sensor Number ");
        Serial.print(sensorNumber);
        Serial.print(" Calibrated value is: ");
        Serial.print(calibratedValue);
        Serial.println(" cm");
        shouldRetakeBaseline = false;
      }

      // Hold the active state for a length of PROXIMITY_PAUSE
      if (millis() - lastActiveProximity >= PROXIMITY_PAUSE) {
        if (distance_ < calibratedValue - 20) {
            lastActiveProximity = millis();
            detected = true;
            Serial.print(sensorNumber);
            Serial.print(" is now active at time ");
            Serial.print(lastActiveProximity);
            Serial.print(" with a distance of ");
            Serial.println(distance_); 
        } else {
            detected = false;
        }
    }
} 

    void start(void (*callback) ()) {
      int currentDistance = calculateDistance();
      compareCurrentWithBaseline();
      if (callback != nullptr) {
        callback();
      }
    }
};


// Enum indicating which direction the door is on from the perspective of the system. Default value will be left.
enum class DoorDirection {
  LEFT,
  RIGHT
};

// Class for the button that can be pressed to recalibrate the ultrasonic sensors
class Button {
 private:
  int input_pin_;
  void (*interrupt_handler_)() = nullptr;
 public:
  Button(int input_pin_) : input_pin_(input_pin_) {}
  Button(int input_pin_, void (*handler)()) {
    this->input_pin_ = input_pin_;
    interrupt_handler_ = handler; 
   }
  void SetISR(void (*handler)()) {
    interrupt_handler_ = handler; 
   }
  void Init() {
     if (interrupt_handler_ == nullptr) {
      Serial.println("Button ISR was never set");
      return;
     }
     pinMode(input_pin_, INPUT_PULLDOWN);
     attachInterrupt(input_pin_, interrupt_handler_, RISING);
  }
};


BluetoothMessenger* bluetoothMessenger = BluetoothMessenger::getInstance();

// First sensor will be the left sensor
UltrasonicSensor firstUltrasonicSensor(FIRST_TRIG_PIN, FIRST_ECHO_PIN, ULTRASONIC_ONE);

// Second sensor will be the right sensor
UltrasonicSensor secondUltrasonicSensor(SECOND_TRIG_PIN, SECOND_ECHO_PIN, ULTRASONIC_TWO);

// Default door direction setting is assumed to be left (door is to the left of the subsystem)
DoorDirection direction = DoorDirection::LEFT;

// Interrupt configured for the button, on rising edge when the button is pressed
// Provides a mechanism to recalibrate the distance of the ultrasonic sensors and switch alternate the DoorDirection enum
// In the future, it should be a more user friendly interaction
void IRAM_ATTR ButtonPressISR() {
  firstUltrasonicSensor.shouldRecalibrate();
  secondUltrasonicSensor.shouldRecalibrate();
  switch(direction) {
    case DoorDirection::LEFT : {
      direction = DoorDirection::RIGHT;
      break;
    }
    case DoorDirection::RIGHT : {
      direction = DoorDirection::LEFT;
      break;
    }
  }
}

Button button(32, ButtonPressISR);

void updateLED(void* parameter) {
  while (true) {
   if (roomOccupationInstance->isOccupied()) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
    }
  }
  delay(50);
}

////////////////////////////////////////////////////////////////////////////////////////
// START OF SETUP AND LOOP
////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.print("Serial port setup complete");
  button.Init();

  pinMode(CONNECTION_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  xTaskCreatePinnedToCore(updateLED, "STATUS_LED", 10000, NULL, 0, NULL, 0);

  // Initializing BLE on the subsystem with the title "ESP32 Subsystem"
  BLEDevice::init("ESP32 Subsystem");
  scanner = BLEDevice::getScan();

  // Set the implemented interface for the advertising so that we can receive the scan result callback properly
  scanner->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallback());
  scanner->setActiveScan(true);

  for (int i = 0; i < NUM_SYSTEMS; i++) {
      shouldAttemptConnect = false;
      startScan();
  }
  bluetoothMessenger->initializeMessageService();
}

void loop() {
  if (connectionsManager->shouldStart()) {
    bluetoothMessenger->startMessageService();
    firstUltrasonicSensor.start(nullptr);
    delay(100);
    secondUltrasonicSensor.start(nullptr);

    if (firstUltrasonicSensor.isActive() && secondUltrasonicSensor.isActive()) {
      switch (direction) {
        // When the door is to the left of the subsystem
        case DoorDirection::LEFT: {
            // If the left most ultrasonic was active before the right most ultrasonic (person is entering the room)
            if (firstUltrasonicSensor.getDetectionTime() < secondUltrasonicSensor.getDetectionTime()) {
              roomOccupationInstance->setOccupied();
              
              // Right most ultrasonic was active before the left ultrasonic (person is leaving the room)
            } else if (firstUltrasonicSensor.getDetectionTime() > secondUltrasonicSensor.getDetectionTime()) {
              roomOccupationInstance->setUnoccupied();
            }
            break;
          }
        // When the door is to the right of the subsystem
        case DoorDirection::RIGHT: {
            // When the left ultrasonic is active before the right ultrasonic (person is leaving the room)
            if (firstUltrasonicSensor.getDetectionTime() < secondUltrasonicSensor.getDetectionTime()) {
              roomOccupationInstance->setUnoccupied();

              // When the right ultrasonic is active before the left ultrasonic (person is entering the room)
            } else if (firstUltrasonicSensor.getDetectionTime() > secondUltrasonicSensor.getDetectionTime()) {
              roomOccupationInstance->setOccupied();
            }
            break;
          }
      }
      //connectionsManager->setValuesForCharacteristics(roomOccupationInstance->getRoomStatus());
    }
  } else {
    // Implement the retry functionality here
    bluetoothMessenger->stopMessageService();
    retryConnection();
  }
}
