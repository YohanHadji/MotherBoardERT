#include <Arduino.h>
#include <Capsule.h>  
#include <Adafruit_NeoPixel.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "rotatorControl.h"

#define CMD_MANUAL_MODE false

#define UI_PORT Serial
#define UI_BAUD 115200

#define RF_AV_UP_PORT Serial8
#define RF_AV_UP_BAUD 115200

#define RF_AV_DOWN_PORT Serial7
#define RF_AV_DOWN_BAUD 115200

#define RF_GSE_PORT Serial6
#define RF_GSE_BAUD 115200

#define ROTATOR_PORT Serial1
#define ROTATOR_BAUD 19200

#define NEOPIXEL_A_PIN 33
#define NEOPIXEL_B_PIN 32

void handleRF_AV_UP(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_AV_DOWN(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_GSE(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel ledA(1, NEOPIXEL_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
Adafruit_NeoPixel ledB(1, NEOPIXEL_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

CapsuleStatic RF_AV_UP(handleRF_AV_UP);
CapsuleStatic RF_AV_DOWN(handleRF_AV_DOWN);
CapsuleStatic RF_GSE(handleRF_GSE);
CapsuleStatic Ui(handleUi);
CapsuleStatic Rotator(handleRotator);
CapsuleStatic Binoculars(handleBinoculars);

static bool positionIsUpdated = false;

static PacketAV_downlink lastPacket;

uint32_t colors[] = {
    0x000000,
    0x32A8A0,
    0x0000FF,
    0xFFEA00,
    0x00FF00,
    0xFF0000,
    0xCF067C,
    0xFF0800
}; 

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  // put your setup code here, to run once:
  RF_AV_UP_PORT.begin(RF_AV_UP_BAUD);
  RF_AV_DOWN_PORT.begin(RF_AV_DOWN_BAUD);

  UI_PORT.begin(115200);
  ROTATOR_PORT.begin(ROTATOR_BAUD);
  // UI_PORT.println("Starting...");

  { 
    ledA.begin();
    ledA.setBrightness(50);
    uint32_t ledColor = colors[random(0,8)];
    ledA.fill(ledColor);
    ledA.show();
    ledA.show();
  }

  { 
    ledB.begin();
    ledB.setBrightness(50);
    uint32_t ledColor = colors[random(0,8)];
    ledB.fill(ledColor);
    ledB.show();
    ledB.show();
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  while (RF_AV_UP_PORT.available()) {
    RF_AV_UP.decode(RF_AV_UP_PORT.read());
  }

  while (RF_AV_DOWN_PORT.available()) {
    RF_AV_DOWN.decode(RF_AV_DOWN_PORT.read());
  }

  while (RF_GSE_PORT.available()) {
    RF_GSE.decode(RF_GSE_PORT.read());
  }

  while (UI_PORT.available()) {
    Ui.decode(UI_PORT.read());
  } 

  if (positionIsUpdated) {
    // positionIsUpdated = false;
    // double payerneLat = 46.8130919;
    // double payerneLon = 6.9435166;
    // double payerneAlt = 540.5;
    // double currentLat = lastPacket.latitude;
    // double currentLon = lastPacket.longitude;
    // double currentAlt = lastPacket.altitude;
    // rotatorCommand newCmd;
    // newCmd = computeRotatorCommand(payerneLat, payerneLon, payerneAlt, currentLat, currentLon, currentAlt);
    // String newOutput;
    // newOutput = "";
    // newOutput += "AZ";
    // newOutput += String(newCmd.azm);
    // newOutput += " EL";
    // newOutput += String(newCmd.elv);
    // ROTATOR_PORT.println(newOutput);
  }
}

void handleRF_AV_UP(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
}

void handleRF_AV_DOWN(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::AV_TELEMETRY:
    {
      // UI_PORT.println("Packet with ID 00 from RF_AV_DOWN received : ");
      uint32_t ledColor = colors[random(sizeof(colors)/sizeof(uint32_t))];
      ledB.fill(ledColor);
      ledB.show();

      memcpy(&lastPacket, dataIn, packetAV_downlink_size);

      uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
      UI_PORT.write(packetToSend,Ui.getCodedLen(len));
      delete[] packetToSend;

      positionIsUpdated = true;
    }
    break;
    default:
    break;
  }
}

void handleRF_GSE(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
  UI_PORT.write(packetToSend,Ui.getCodedLen(len));
  delete[] packetToSend;
}

void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  if (packetId>CAPSULE_ID::BEGIN_AV_UP_ID & packetId<CAPSULE_ID::END_AV_UP_ID) {
    uint8_t* packetToSend = RF_AV_UP.encode(packetId,dataIn,len);
    RF_AV_UP_PORT.write(packetToSend,RF_AV_UP.getCodedLen(len));
    delete[] packetToSend;
  }
  else if (packetId>CAPSULE_ID::BEGIN_GSE_UP_ID & packetId<CAPSULE_ID::END_GSE_UP_ID) {
    uint8_t* packetToSend = RF_GSE.encode(packetId,dataIn,len);
    RF_GSE_PORT.write(packetToSend,RF_GSE.getCodedLen(len));
    delete[] packetToSend;
  }
}

void handleRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}

void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}