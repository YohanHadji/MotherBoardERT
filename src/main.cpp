#include <Arduino.h>
#include <Capsule.h>  
#include <Adafruit_NeoPixel.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "rotator.h"

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

static rotClass rotator;

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

  if (rotator.isUpdated()) {
    rotator.setMode(rotator.computeMode());
    rotatorCommand cmdToSend;
    cmdToSend = rotator.computeCommand();
    PacketTrackerCmd packetToSend;
    packetToSend.azm = cmdToSend.azm;
    packetToSend.elv = cmdToSend.elv;
    packetToSend.mode = rotator.getMode();

    byte* buffer = new byte[packetTrackerCmdSize]; // Allocate memory for the byte array
    memcpy(buffer, &packetToSend, packetTrackerCmdSize); // Copy the struct to the byte array

    uint8_t* bytesToSend = Rotator.encode(CAPSULE_ID::TRACKER_CMD,buffer,packetTrackerCmdSize);
    ROTATOR_PORT.write(bytesToSend,Rotator.getCodedLen(packetTrackerCmdSize));
    delete[] bytesToSend;
    delete[] buffer;
  }
}

void handleRF_AV_DOWN(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::AV_TELEMETRY:
    {
      // UI_PORT.println("Packet with ID 00 from RF_AV_DOWN received : ");
      uint32_t ledColor = colors[random(sizeof(colors)/sizeof(uint32_t))];
      ledB.fill(ledColor);
      ledB.show();

      PacketAV_downlink lastPacket;
      memcpy(&lastPacket, dataIn, packetAV_downlink_size);
      rotator.updateAV(lastPacket);

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
  switch(packetId) {
    case CAPSULE_ID::GSE_TELEMETRY:
    {
      uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
      UI_PORT.write(packetToSend,Ui.getCodedLen(len));
      delete[] packetToSend;
    }
    break;
    default:
    break;
  }
}

void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  if (packetId>CAPSULE_ID::BEGIN_AV_UP_ID and packetId<CAPSULE_ID::END_AV_UP_ID) {
    uint8_t* packetToSend = RF_AV_UP.encode(packetId,dataIn,len);
    RF_AV_UP_PORT.write(packetToSend,RF_AV_UP.getCodedLen(len));
    delete[] packetToSend;
  }
  else if (packetId>CAPSULE_ID::BEGIN_GSE_UP_ID and packetId<CAPSULE_ID::END_GSE_UP_ID) {
    uint8_t* packetToSend = RF_GSE.encode(packetId,dataIn,len);
    RF_GSE_PORT.write(packetToSend,RF_GSE.getCodedLen(len));
    delete[] packetToSend;
  }
  switch (packetId) {
    case CAPSULE_ID::CALIBRATE_TELEMETRY:
      rotator.calibrateTelemetry();
    break;
    default:
    break;
  }
}

void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch(packetId) {
    case CAPSULE_ID::BINOC_ATTITUDE:
      PacketBinocAttitude binocAttitude;
      memcpy(&binocAttitude, dataIn, packetBinocAttitudeSize);
      rotator.updateBinocAttitude(binocAttitude);
    break;
    case CAPSULE_ID::BINOC_POSITION:
      PacketBinocPosition binocPosition;
      memcpy(&binocPosition, dataIn, packetBinocPositionSize);
      rotator.updateBinocPosition(binocPosition);
    break;
    case CAPSULE_ID::BINOC_STATUS:
      PacketBinocStatus binocStatus;
      memcpy(&binocStatus, dataIn, packetBinocStatusSize);
      rotator.updateBinocStatus(binocStatus);
    break;
    case CAPSULE_ID::BINOC_GLOBAL_STATUS:
      PacketBinocGlobalStatus binocGlobalStatus;
      memcpy(&binocGlobalStatus, dataIn, packetBinocGlobalStatusSize);
      rotator.updateBinocGlobalStatus(binocGlobalStatus);
    break;
    default:
    break;
  }
}

// These two little guys don't have anything to handle because the rotator and av up 
// are not sending anything back to the motherboard 

void handleRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}

void handleRF_AV_UP(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}