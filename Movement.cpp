#Movement
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// BLE Service and Characteristic UUIDs (must match your web controller)
#define SERVICE_UUID        "00001234-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00005678-0000-1000-8000-00805f9b34fb"

// Define servo pins for each leg (hip and knee)
const int FR_HIP_PIN = 13;   // Front Right Hip
const int FR_KNEE_PIN = 12;  // Front Right Knee
const int FL_HIP_PIN = 14;   // Front Left Hip
const int FL_KNEE_PIN = 27;  // Front Left Knee
const int BR_HIP_PIN = 26;   // Back Right Hip
const int BR_KNEE_PIN = 25;  // Back Right Knee
const int BL_HIP_PIN = 33;   // Back Left Hip
const int BL_KNEE_PIN = 32;  // Back Left Knee

// Create servo objects
Servo frHip, frKnee, flHip, flKnee, brHip, brKnee, blHip, blKnee;

// Servo positions (adjust these for your specific robot)
const int HIP_CENTER = 90;
const int HIP_FORWARD = 60;
const int HIP_BACKWARD = 120;
const int KNEE_UP = 120;
const int KNEE_DOWN = 60;

// Movement timing
const int STEP_DELAY = 300; // Time between steps in ms

// BLE Variables
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Command definitions
#define CMD_FORWARD  0x01
#define CMD_BACKWARD 0x02
#define CMD_LEFT     0x03
#define CMD_RIGHT    0x04

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
      // Restart advertising to allow reconnection
      pServer->getAdvertising()->start();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        uint8_t command = value[0];
        Serial.print("Received command: ");
        Serial.println(command, HEX);
        
        switch(command) {
          case CMD_FORWARD:
            moveForward(1);
            break;
          case CMD_BACKWARD:
            moveBackward(1);
            break;
          case CMD_LEFT:
            turnLeft(1);
            break;
          case CMD_RIGHT:
            turnRight(1);
            break;
          default:
            Serial.println("Unknown command");
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach servos to pins
  frHip.attach(FR_HIP_PIN);
  frKnee.attach(FR_KNEE_PIN);
  flHip.attach(FL_HIP_PIN);
  flKnee.attach(FL_KNEE_PIN);
  brHip.attach(BR_HIP_PIN);
  brKnee.attach(BR_KNEE_PIN);
  blHip.attach(BL_HIP_PIN);
  blKnee.attach(BL_KNEE_PIN);
  
  // Initialize servos to default position
  resetAllServos();
  delay(1000);

  // Create the BLE Device
  BLEDevice::init("QuadBot");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  
  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  // Handle disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // Handle new connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  delay(10); // Small delay to reduce CPU usage
}

void resetAllServos() {
  frHip.write(HIP_CENTER);
  frKnee.write(KNEE_UP);
  flHip.write(HIP_CENTER);
  flKnee.write(KNEE_UP);
  brHip.write(HIP_CENTER);
  brKnee.write(KNEE_UP);
  blHip.write(HIP_CENTER);
  blKnee.write(KNEE_UP);
}

void moveForward(int steps) {
  for (int i = 0; i < steps; i++) {
    // Step 1: Move front right and back left legs forward
    frHip.write(HIP_FORWARD);
    blHip.write(HIP_FORWARD);
    delay(STEP_DELAY/2);
    
    // Lift these legs
    frKnee.write(KNEE_UP);
    blKnee.write(KNEE_UP);
    delay(STEP_DELAY/2);
    
    // Move body forward by pushing with other legs
    flHip.write(HIP_BACKWARD);
    brHip.write(HIP_BACKWARD);
    delay(STEP_DELAY);
    
    // Lower the lifted legs
    frKnee.write(KNEE_DOWN);
    blKnee.write(KNEE_DOWN);
    delay(STEP_DELAY/2);
    
    // Center the moved legs
    frHip.write(HIP_CENTER);
    blHip.write(HIP_CENTER);
    delay(STEP_DELAY/2);
    
    // Step 2: Move front left and back right legs forward
    flHip.write(HIP_FORWARD);
    brHip.write(HIP_FORWARD);
    delay(STEP_DELAY/2);
    
    // Lift these legs
    flKnee.write(KNEE_UP);
    brKnee.write(KNEE_UP);
    delay(STEP_DELAY/2);
    
    // Move body forward by pushing with other legs
    frHip.write(HIP_BACKWARD);
    blHip.write(HIP_BACKWARD);
    delay(STEP_DELAY);
    
    // Lower the lifted legs
    flKnee.write(KNEE_DOWN);
    brKnee.write(KNEE_DOWN);
    delay(STEP_DELAY/2);
    
    // Center the moved legs
    flHip.write(HIP_CENTER);
    brHip.write(HIP_CENTER);
    delay(STEP_DELAY/2);
  }
}

void moveBackward(int steps) {
  for (int i = 0; i < steps; i++) {
    // Step 1: Move front right and back left legs backward
    frHip.write(HIP_BACKWARD);
    blHip.write(HIP_BACKWARD);
    delay(STEP_DELAY/2);
    
    // Lift these legs
    frKnee.write(KNEE_UP);
    blKnee.write(KNEE_UP);
    delay(STEP_DELAY/2);
    
    // Move body backward by pushing with other legs
    flHip.write(HIP_FORWARD);
    brHip.write(HIP_FORWARD);
    delay(STEP_DELAY);
    
    // Lower the lifted legs
    frKnee.write(KNEE_DOWN);
    blKnee.write(KNEE_DOWN);
    delay(STEP_DELAY/2);
    
    // Center the moved legs
    frHip.write(HIP_CENTER);
    blHip.write(HIP_CENTER);
    delay(STEP_DELAY/2);
    
    // Step 2: Move front left and back right legs backward
    flHip.write(HIP_BACKWARD);
    brHip.write(HIP_BACKWARD);
    delay(STEP_DELAY/2);
    
    // Lift these legs
    flKnee.write(KNEE_UP);
    brKnee.write(KNEE_UP);
    delay(STEP_DELAY/2);
    
    // Move body backward by pushing with other legs
    frHip.write(HIP_FORWARD);
    blHip.write(HIP_FORWARD);
    delay(STEP_DELAY);
    
    // Lower the lifted legs
    flKnee.write(KNEE_DOWN);
    brKnee.write(KNEE_DOWN);
    delay(STEP_DELAY/2);
    
    // Center the moved legs
    flHip.write(HIP_CENTER);
    brHip.write(HIP_CENTER);
    delay(STEP_DELAY/2);
  }
}

void turnRight(int steps) {
  for (int i = 0; i < steps; i++) {
    // Step 1: Move right side legs forward, left side backward
    frHip.write(HIP_FORWARD);
    brHip.write(HIP_FORWARD);
    flHip.write(HIP_BACKWARD);
    blHip.write(HIP_BACKWARD);
    delay(STEP_DELAY/2);
    
    // Lift all legs
    frKnee.write(KNEE_UP);
    flKnee.write(KNEE_UP);
    brKnee.write(KNEE_UP);
    blKnee.write(KNEE_UP);
    delay(STEP_DELAY/2);
    
    // Move body by pushing with all legs
    frHip.write(HIP_BACKWARD);
    brHip.write(HIP_BACKWARD);
    flHip.write(HIP_FORWARD);
    blHip.write(HIP_FORWARD);
    delay(STEP_DELAY);
    
    // Lower all legs
    frKnee.write(KNEE_DOWN);
    flKnee.write(KNEE_DOWN);
    brKnee.write(KNEE_DOWN);
    blKnee.write(KNEE_DOWN);
    delay(STEP_DELAY/2);
    
    // Center all legs
    frHip.write(HIP_CENTER);
    flHip.write(HIP_CENTER);
    brHip.write(HIP_CENTER);
    blHip.write(HIP_CENTER);
    delay(STEP_DELAY/2);
  }
}

void turnLeft(int steps) {
  for (int i = 0; i < steps; i++) {
    // Step 1: Move left side legs forward, right side backward
    flHip.write(HIP_FORWARD);
    blHip.write(HIP_FORWARD);
    frHip.write(HIP_BACKWARD);
    brHip.write(HIP_BACKWARD);
    delay(STEP_DELAY/2);
    
    // Lift all legs
    frKnee.write(KNEE_UP);
    flKnee.write(KNEE_UP);
    brKnee.write(KNEE_UP);
    blKnee.write(KNEE_UP);
    delay(STEP_DELAY/2);
    
    // Move body by pushing with all legs
    flHip.write(HIP_BACKWARD);
    blHip.write(HIP_BACKWARD);
    frHip.write(HIP_FORWARD);
    brHip.write(HIP_FORWARD);
    delay(STEP_DELAY);
    
    // Lower all legs
    frKnee.write(KNEE_DOWN);
    flKnee.write(KNEE_DOWN);
    brKnee.write(KNEE_DOWN);
    blKnee.write(KNEE_DOWN);
    delay(STEP_DELAY/2);
    
    // Center all legs
    frHip.write(HIP_CENTER);
    flHip.write(HIP_CENTER);
    brHip.write(HIP_CENTER);
    blHip.write(HIP_CENTER);
    delay(STEP_DELAY/2);
  }
}
