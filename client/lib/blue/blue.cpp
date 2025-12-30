#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <config.h>
#include "blue.h"

Control* control = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("client connected.");
    control = new Control();
  }

  void onDisconnect(BLEServer* pServer) override {
  Serial.println("client disconnected.");

  if (control != nullptr) {
    delete control;
    control = nullptr;
  }

  BLEAdvertising* adv = pServer->getAdvertising();
  adv->start();
  Serial.println("advertising started again");
}

};

class BLECommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    std::string value = pChar->getValue();
    pChar->setValue(value);
    pChar->notify();
    Serial.println("recieved: " + String(value.c_str()));
    
    control->funcMap[value.c_str()]();
  }
};

class BLETargetCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {

    std::string value = pChar->getValue();
    String jsonStr = value.c_str();
    Serial.println("received: " + jsonStr);

    JsonDocument doc;
    auto error = deserializeJson(doc, jsonStr);
    if (error) {
      Serial.println("JSON Parse Error");
      return;
    }

    for (JsonPair kv : doc.as<JsonObject>()) {
      const char* key = kv.key().c_str();
      JsonVariant val = kv.value();

      if (control->targetMap.count(key)) {
        float* target = control->targetMap[key];
        *target = val.as<float>();
        continue;
      }
    }


    for (const auto& kv : control->targetMap) {
        const char* key = kv.first.c_str(); // Get the C-style string key
        float* valuePtr = kv.second;        // Get the float pointer
        
        Serial.print("  [");
        Serial.print(key);
        Serial.print("]: ");
        
        // Check if the pointer is valid before dereferencing
        if (valuePtr != nullptr) {
            // Dereference the pointer (*) to print the actual float value
            Serial.println(*valuePtr, 4); // Print float with 4 decimal places
        } else {
            Serial.println("ERROR (Value is nullptr)");
        }
    }

    pChar->setValue(value);
    pChar->notify();
  }
};

class BLESettingsCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {

    std::string value = pChar->getValue();
    String jsonStr = value.c_str();

    JsonDocument doc;
    auto error = deserializeJson(doc, jsonStr);
    if (error) {
      Serial.println("JSON Parse Error");
      return;
    }

    for (JsonPair kv : doc.as<JsonObject>()) {
      const char* key = kv.key().c_str();
      JsonVariant val = kv.value();

      if (control->settingsMap.count(key)) {
        float* target = control->settingsMap[key];
        *target = val.as<float>();
        continue;
      }
    }

  for (const auto& kv : control->settingsMap) {
        const char* key = kv.first.c_str(); // Get the C-style string key
        float* valuePtr = kv.second;        // Get the float pointer
        
        Serial.print("  [");
        Serial.print(key);
        Serial.print("]: ");
        
        // Check if the pointer is valid before dereferencing
        if (valuePtr != nullptr) {
            // Dereference the pointer (*) to print the actual float value
            Serial.println(*valuePtr, 4); // Print float with 4 decimal places
        } else {
            Serial.println("ERROR (Value is nullptr)");
        }
    }

    pChar->setValue(value);
    pChar->notify();
  }
};

void initBLE() {
  BLEDevice::init(BLE_NAME.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // initialize service and characteristic
  BLEService *pService = pServer->createService(SERVICE_UUID.c_str());
  BLECharacteristic *pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_CHARACTERISTIC_UUID.c_str(),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  BLECharacteristic *pTargetCharacteristic = pService->createCharacteristic(
    TARGET_CHARACTERISTIC_UUID.c_str(),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  BLECharacteristic *pSettingsCharacteristic = pService->createCharacteristic(
    SETTINGS_CHARACTERISTIC_UUID.c_str(),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
 
  pCommandCharacteristic->setValue(BLE_INIT_VALUE);
  pCommandCharacteristic->setCallbacks(new BLECommandCallbacks());
  pCommandCharacteristic->addDescriptor(new BLE2902());
  pTargetCharacteristic->setValue(BLE_INIT_VALUE);
  pTargetCharacteristic->setCallbacks(new BLETargetCallbacks());
  pTargetCharacteristic->addDescriptor(new BLE2902());
  pSettingsCharacteristic->setValue(BLE_INIT_VALUE);
  pSettingsCharacteristic->setCallbacks(new BLESettingsCallbacks());
  pSettingsCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  BLEAdvertisementData advertisingData;
  advertisingData.setName(BLE_NAME);  
  advertisingData.setCompleteServices(BLEUUID(SERVICE_UUID));

  pAdvertising->setAdvertisementData(advertisingData);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  BLEDevice::startAdvertising();
  Serial.println("waiting for client connection...");
}