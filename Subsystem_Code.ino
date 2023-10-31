#include "BLEDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <string.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SUBSYSTEM_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"

#define FIRST_TRIG_PIN 2
#define FIRST_ECHO_PIN 15
#define SECOND_TRIG_PIN 25
#define SECOND_ECHO_PIN 33
#define CONNECTION_LED_PIN 13
#define STATUS_LED_PIN 0

#define PROXIMITY_PAUSE 3000

#define ULTRASONIC_ONE 1
#define ULTRASONIC_TWO 2

BLEScan* scanner;
BLEAdvertisedDevice targetDevice;
BLEClient* client;

// BLECharacteristic that we will be writing to
BLERemoteCharacteristic* characteristic;

bool shouldAttemptConnect = false;
// If connection to the main ESP32 succeeds, this will be set to true
bool shouldStart = false;

bool connectToServer(BLEAdvertisedDevice* device) {
  Serial.println("Beginning BLE connection with main ESP32...");
  // Obtain the client handle
  client = BLEDevice::createClient();
  if (client == nullptr) {
    Serial.print("Error establishing BLEClient");
    return false;
  } else {
    Serial.println("BLEClient success");
  }
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
  std::string characteristicString = characteristic->toString();
  Serial.print("Connected to characteristic: ");
  Serial.println(characteristicString.c_str());
  return true;
}


class AdvertisedDeviceCallback : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override;
};

void AdvertisedDeviceCallback::onResult(BLEAdvertisedDevice advertisedDevice) {
  Serial.println(advertisedDevice.getName().c_str());
  if (advertisedDevice.getName() == std::string("ESP32")) {
    Serial.println("Located main ESP32");
    BLEAddress targetAddress = advertisedDevice.getAddress();
    std::string addressString = targetAddress.toString();
    Serial.print("ESP32 Address: ");
    Serial.println(addressString.c_str());
    targetDevice = advertisedDevice;
    shouldAttemptConnect = true;
  }
}

class MyClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* client) override;
    void onDisconnect(BLEClient* client) override;
};

void MyClientCallbacks::onConnect(BLEClient* client) {

}

void MyClientCallbacks::onDisconnect(BLEClient* client) {
  //retryConnection();
  shouldStart = false;
}

void startScan() {
  Serial.println("Starting BLE scan...");
  scanner->start(15);
  if (shouldAttemptConnect) {
    shouldStart = connectToServer(&targetDevice);
    if (shouldStart) {
      // Turn the LED on to indicate connection was successful
      digitalWrite(CONNECTION_LED_PIN, HIGH);
      if (client != nullptr) {
        client->setClientCallbacks(new MyClientCallbacks());
      }
    }
  }
}

void clearClientInfo() {
  client = nullptr;
  characteristic = nullptr;
  shouldAttemptConnect = false;
  
}

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

void sendBluetoothMessage(String message, BLERemoteCharacteristic* characteristic) {
  uint8_t data[message.length() + 1];
  memcpy(data, message.c_str(), message.length());
  characteristic->writeValue(data, message.length());
}


class UltrasonicStatusListener {
  public:
  virtual void onUltrasonicStatusChanged(bool newStatus, int sensorNumber) = 0;
};

class ImplementedListener : public UltrasonicStatusListener {
  public:
  void onUltrasonicStatusChanged(bool newStatus, int sensorNumber) override;
};

void ImplementedListener::onUltrasonicStatusChanged(bool newStatus, int sensorNumber) {
  Serial.print("Status changed for ultrasonic ");
  Serial.print(sensorNumber);
  Serial.print(": ");
  Serial.println(newStatus);
}

////////////////////////////////////////////////////////////////////////////////////////
// Proximity sensor class definition/parameters
// Trig pin goes to GIOP 2
// Echo pin goes to GIOP 15
////////////////////////////////////////////////////////////////////////////////////////
class UltrasonicSensor {
  private:
    UltrasonicStatusListener* listener;
    int trigger_pin;
    int echo_pin;
    long duration;
    int distance_;
    int calibratedValue = 0;
    bool isCalibrated = false;
    long unsigned int lastActiveProximity = 0;
    bool proximityLockLow = true;
    bool proximityTakeLowTime;
    long unsigned int proximityLowIn;
    int sensorNumber;

    // Boolean that indicates status of the ultrasonic sensor (active or nonactive)
    bool detected = false;
    bool previousStatus = false;

    // Flag that is set to true whenenver the vibration sensor is active
    // When the system is moved, we need to recalibrate the ultrasonic's baseline
    static bool shouldRetakeBaseline;

  public:
    UltrasonicSensor(int trig_pin, int echo_pin, int sensor_number) {
      pinMode(trig_pin, OUTPUT);
      pinMode(echo_pin, INPUT);
      trigger_pin = trig_pin;
      this->echo_pin = echo_pin;
      sensorNumber = sensor_number;
    }

    void setListener(UltrasonicStatusListener* listener) {
      this->listener = listener;
    }
    int distance() {
      return distance_;
    }

    bool isActive() {
      return detected;
    }

    static void setBaselineFlag(bool flag) {
      shouldRetakeBaseline = flag;
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
        Serial.println("Calibrated value is: ");
        Serial.print(calibratedValue);
        Serial.print(" cm");
      }
      if (distance_ < calibratedValue - 30 || distance_ > calibratedValue + 30) {
        lastActiveProximity = millis();
        detected = true;
        if (proximityLockLow) {
          //makes sure we wait for a transition to LOW before any further output is made:
          proximityLockLow = false;
          delay(50);
        }
        proximityTakeLowTime = true;
      }
      else {
        if (proximityTakeLowTime) {
          proximityLowIn = millis();
          proximityTakeLowTime = false;
        }
        if (!proximityLockLow && millis() - proximityLowIn > PROXIMITY_PAUSE) {
          proximityLockLow = true;
          detected = false;
        }
      }
      if (previousStatus != detected) {
        listener->onUltrasonicStatusChanged(detected, sensorNumber);
      }
    }

    void start() {
      int currentDistance = calculateDistance();
      compareCurrentWithBaseline();
    }
};
bool UltrasonicSensor::shouldRetakeBaseline = false;



// First sensor will be the left sensor
UltrasonicSensor* firstUltrasonicSensor;

// Second sensor will be the right sensor
UltrasonicSensor* secondUltrasonicSensor;

// Enum indicating which direction the door is on from the perspective of the system. Default value will be left.
enum DoorDirection {
  LEFT,
  RIGHT
};

DoorDirection direction = DoorDirection::LEFT;

// Main global boolean variable that we will write to the BLE characteristic to indicate the room's occupation status (person is present/absent)
bool isRoomOccupied = false;
String roomStatus = "";


void updateLED(void* parameter) {
  while (true) {
   if (isRoomOccupied) {
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
  firstUltrasonicSensor = new UltrasonicSensor(FIRST_TRIG_PIN, FIRST_ECHO_PIN, ULTRASONIC_ONE);
  secondUltrasonicSensor = new UltrasonicSensor(SECOND_TRIG_PIN, SECOND_ECHO_PIN, ULTRASONIC_TWO);
  firstUltrasonicSensor->setListener(new ImplementedListener());
  secondUltrasonicSensor->setListener(new ImplementedListener());

  pinMode(CONNECTION_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  xTaskCreatePinnedToCore(updateLED, "STATUS_LED", 10000, NULL, 0, NULL, 0);

  BLEDevice::init("ESP32 Subsystem");
  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallback());
  scanner->setActiveScan(true);
  startScan();
}

void loop() {
  if (shouldStart) {
    firstUltrasonicSensor->start();
    secondUltrasonicSensor->start();
    if (firstUltrasonicSensor->isActive()) {
      Serial.println("Ultrasonic 1 currently active");
    }
    if (secondUltrasonicSensor->isActive()) {
      Serial.println("Ultrasonic 2 currently active");
    }

    if (firstUltrasonicSensor->isActive() && secondUltrasonicSensor->isActive()) {
      switch (direction) {
        case LEFT: {
            // Person is entering the room
            if (firstUltrasonicSensor->getDetectionTime() < secondUltrasonicSensor->getDetectionTime()) {
              isRoomOccupied = true;
              roomStatus = "Occupied";

              // Person is exiting the room
            } else if (firstUltrasonicSensor->getDetectionTime() > secondUltrasonicSensor->getDetectionTime()) {
              isRoomOccupied = false;
              roomStatus = "Not occupied";
            }
            Serial.println(roomStatus.c_str());
            break;
          }
        case RIGHT: {

            // Same idea for the LEFT case, but just switch everything around
            if (firstUltrasonicSensor->getDetectionTime() < secondUltrasonicSensor->getDetectionTime()) {
              isRoomOccupied = false;
              roomStatus = "Not occupied";

              // Person is exiting the room
            } else if (firstUltrasonicSensor->getDetectionTime() > secondUltrasonicSensor->getDetectionTime()) {
              isRoomOccupied = true;
              roomStatus = "Occupied";
            }
            Serial.println(roomStatus.c_str());
            break;
          }
          //characteristic->writeValue(static_cast<uint8_t>(isRoomOccupied));

      }
      sendBluetoothMessage(roomStatus, characteristic);
    }
  } else {
    // Implement the retry functionality here
    retryConnection();
  }
}
