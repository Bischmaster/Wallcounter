
#include <Arduino.h>
#include <ArduinoBLE.h>

#define PIEZO_PIN A0
#define CALIBRATION_TIME 10000

BLEService wallballService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEByteCharacteristic countChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);
BLEByteCharacteristic resetChar("19b10002-e8f2-537e-4f6c-d104768a1214", BLEWrite);
BLEByteCharacteristic calibrateChar("19b10003-e8f2-537e-4f6c-d104768a1214", BLEWrite);
BLEUnsignedShortCharacteristic thresholdChar("19b10004-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify);

uint8_t count = 0;
bool calibrating = false;
unsigned long calibrationStart = 0;
int maxSignal = 0;
int threshold = 300;

void setup() {
  Serial.begin(115200);
  pinMode(PIEZO_PIN, INPUT);

  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1);
  }

  BLE.setLocalName("WallballCounter");
  BLE.setDeviceName("WallballCounter");
  BLE.setAdvertisedService(wallballService);

  wallballService.addCharacteristic(countChar);
  wallballService.addCharacteristic(resetChar);
  wallballService.addCharacteristic(calibrateChar);
  wallballService.addCharacteristic(thresholdChar);
  BLE.addService(wallballService);

  countChar.writeValue(count);
  thresholdChar.writeValue(threshold);

  resetChar.setEventHandler(BLEWritten, onReset);
  calibrateChar.setEventHandler(BLEWritten, onCalibrate);

  BLE.advertise();
  Serial.println("BLE gestartet, wartet auf Verbindung...");
}

void loop() {
  BLE.poll();

  if (calibrating) {
    unsigned long now = millis();
    int val = analogRead(PIEZO_PIN);
    if (val > maxSignal) {
      maxSignal = val;
    }
    if (now - calibrationStart >= CALIBRATION_TIME) {
      calibrating = false;
      threshold = maxSignal * 0.7;
      thresholdChar.writeValue(threshold);
      
      Serial.print("Kalibrierung abgeschlossen. Neuer Schwellenwert: ");
      Serial.println(threshold);
    }
  } else {
    int val = analogRead(PIEZO_PIN);
    if (val > threshold) {
      count++;
      countChar.writeValue(count);
      
      delay(300); // debounce
    }
  }
}

void onReset(BLEDevice central, BLECharacteristic characteristic) {
  count = 0;
  countChar.writeValue(count);
  
  Serial.println("Zähler zurückgesetzt.");
}

void onCalibrate(BLEDevice central, BLECharacteristic characteristic) {
  calibrating = true;
  maxSignal = 0;
  calibrationStart = millis();
  Serial.println("Kalibrierung gestartet...");
}
