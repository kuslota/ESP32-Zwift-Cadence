#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// UUIDs for our service and characteristics
#define SERVICE_UUID 0x1816
#define CSC_MEASUREMENT_UUID 0x2A5B
#define CSC_FEATURE_UUID 0x2A5C 
#define SENSOR_LOCATION_UUID 0x2A5D

const int hallPin = 32; // Pin connected to the A3144 Hall effect sensor
volatile uint32_t lastDetectionMillis = 0; // Time of the last detected magnet 
volatile uint16_t crankRevolutions = 0; // Total number of crank revolutions
uint16_t last_crank_event_time = 0; // Time of the last crank event

BLECharacteristic *pCSCMeasurementCharacteristic;

void magnetDetected() {
  uint32_t currentMillis = millis();
  if (currentMillis - lastDetectionMillis > 50) {  // Debounce for 50ms
    crankRevolutions++;

    // Calculate time difference since the last magnet detection in 1/1024th of a second units
    uint16_t timeDifference = (currentMillis - lastDetectionMillis) * 1024 / 1000;
    last_crank_event_time += timeDifference;

    // Adjust for overflow
    if (last_crank_event_time > 65530) {
      last_crank_event_time -= 65530;
    }

    lastDetectionMillis = currentMillis;

    // Print the crank revolution count
    Serial.println("Crank Revolution Detected! Total: " + String(crankRevolutions));

    // Construct and send the BLE message
    uint8_t flags = 0x02;  // Only crank data present
    uint8_t cscMeasurement[5]; // Data for cadence only

    cscMeasurement[0] = flags;
    cscMeasurement[1] = crankRevolutions & 0xFF;
    cscMeasurement[2] = (crankRevolutions >> 8) & 0xFF;
    cscMeasurement[3] = last_crank_event_time & 0xFF;
    cscMeasurement[4] = (last_crank_event_time >> 8) & 0xFF;

    pCSCMeasurementCharacteristic->setValue(cscMeasurement, sizeof(cscMeasurement));
    pCSCMeasurementCharacteristic->notify();
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize BLE
  BLEDevice::init("ESP32 Bike Trainer");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService((uint16_t)SERVICE_UUID);

  // Setup CSC Measurement Characteristic
  pCSCMeasurementCharacteristic = pService->createCharacteristic(
                                      (uint16_t)CSC_MEASUREMENT_UUID,
                                      BLECharacteristic::PROPERTY_NOTIFY
                                    );
  pCSCMeasurementCharacteristic->addDescriptor(new BLE2902());

  // Setup CSC Feature and Sensor Location Characteristics
  pService->createCharacteristic((uint16_t)CSC_FEATURE_UUID, BLECharacteristic::PROPERTY_READ)
          ->setValue((uint8_t*)"\x03\x00", 2); 
  pService->createCharacteristic((uint16_t)SENSOR_LOCATION_UUID, BLECharacteristic::PROPERTY_READ)
          ->setValue((uint8_t*)"\x0C", 1); 

  // Start the BLE service and advertising
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID((uint16_t)SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  // Setup Hall effect sensor
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), magnetDetected, FALLING);
}
void loop() { }
/*
void loop() {
  cumulative_crank_revolutions++;  // Increase by 1 for 60 RPM

  // Increase the last crank event time by 1024 1/1024ths of a second for 60 RPM
  last_crank_event_time += 1024;

  uint8_t flags = 0x02;  // Only crank data

  uint8_t cscMeasurement[5];  // For cadence only

  cscMeasurement[0] = flags;
  cscMeasurement[1] = cumulative_crank_revolutions & 0xFF;
  cscMeasurement[2] = (cumulative_crank_revolutions >> 8) & 0xFF;
  cscMeasurement[3] = last_crank_event_time & 0xFF;
  cscMeasurement[4] = (last_crank_event_time >> 8) & 0xFF;

  pCSCMeasurementCharacteristic->setValue(cscMeasurement, sizeof(cscMeasurement));
  pCSCMeasurementCharacteristic->notify();

  delay(1000);
}
*/