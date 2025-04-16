#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM6DSO.h>
#include <BLEDevice.h>

// LED pin
#define YELLOW_LED 12

// LSM6DSO object
LSM6DSO myIMU;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Duration of calibration
#define CALIBRATION_MS 5000

// Global variables for timing, step detection, and threshold calculation
unsigned long start_calibration;
bool isActive = false;
float threshold = 0;
float rootMeanSquare;
unsigned int steps = 0;
unsigned int prevSteps = 0;

// When the phone writes data to this characteristic, the onWrite() function is triggered
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
 
    // Turns LED ON
    if (value == "ON") {
      digitalWrite(YELLOW_LED, HIGH);
      Serial.print("Light turned on");
    }
    // Turns LED OFF
    else if (value == "OFF") {
      digitalWrite(YELLOW_LED, LOW);
      Serial.print("Light turned off");
    }
    else {
      Serial.print("Input not defined command.");
    }
  }
};

// BLE server, service and characteristic
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

void setup() {
  Serial.begin(115200);

  Serial.print("Initializing LED Light");

  // Initialize LED to OFF
  pinMode(YELLOW_LED, OUTPUT);
  digitalWrite(YELLOW_LED, LOW);
  
  Serial.println("Starting BLE");

  // Initalize LSM6DSO
  Wire.begin();
  if (myIMU.begin()) {
    Serial.println("LSM6DSO Ready");
  }
  else {
    Serial.println("Could not connect to LSM6DSO");
    while(1);
  }

  // Load basic settings for LSM6SO
  if( myIMU.initialize(BASIC_SETTINGS) ) {
    Serial.println("Settings Loaded.");
  }
  
  // Initialize BLE server
  BLEDevice::init("SDSUCS");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  
  // Create characteristic for above server
  pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_READ   |
                                          BLECharacteristic::PROPERTY_WRITE  |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );

  // Allow callbacks as part of characteristics and start service
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Use commands: ON / OFF ");
  pService->start();
  
  // Advertise service
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // Start calibration for timer
  start_calibration = millis();
  Serial.println("Calibration started");

  // Determine threshold using root mean square
  while (millis() - start_calibration < CALIBRATION_MS) {
    rootMeanSquare = float(sqrt(pow(myIMU.readFloatGyroX(), 2) + pow(myIMU.readFloatGyroY(), 2) + pow(myIMU.readFloatGyroZ(), 2)));

    // Keep track of maximum RMS seen during calibration
    threshold = std::max(threshold, rootMeanSquare);
    Serial.println(threshold);

  }
  Serial.println("Calibrated");

  delay(500);
}
 
void loop() {

  // Calculate current RMS
  rootMeanSquare = float(sqrt(pow(myIMU.readFloatGyroX(), 2) + pow(myIMU.readFloatGyroY(), 2) + pow(myIMU.readFloatGyroZ(), 2)));

  // If RMS is above threshold, add a step
  if (rootMeanSquare > threshold) {
    if (!isActive) {
      // Only increment if previously inactive
      steps++;
      isActive = true;
      // Add delay to avoid over counting
      delay(500);
    }
  } else {
    // Reset our active state if RMS falls below threshold
    isActive = false;
  }

  // Print steps to characteristic only when steps change
  Serial.print("Steps: ");
  Serial.println(steps);
  if (prevSteps < steps) {
    Serial.print("Step Count: ");
    Serial.println(steps);
    prevSteps = steps;
    pCharacteristic->setValue(String(steps).c_str());
  }
  delay(20);
}