#include <Arduino.h>
#include <Capsule.h>  
#include <Adafruit_NeoPixel.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "rotator.h"
#include "config.h"

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_AV_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_GSE_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len);

void sendRotatorCmd(rotatorCommand cmd);

Adafruit_NeoPixel ledA(1, NEOPIXEL_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
Adafruit_NeoPixel ledB(1, NEOPIXEL_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

CapsuleStatic RF_UPLINK(handleRF_UPLINK);
CapsuleStatic RF_AV_DOWNLINK(handleRF_AV_DOWNLINK);
CapsuleStatic RF_GSE_DOWNLINK(handleRF_GSE_DOWNLINK);
CapsuleStatic Ui(handleUi);
CapsuleStatic Rotator(handleRotator);
CapsuleStatic Binoculars(handleBinoculars);
CapsuleStatic CommandInput(handleCommandInput);

static bool positionIsUpdated = false;

static rotClass rotator;

uint32_t colors[] = {
    0x32A8A0, // Cyan
    0x0000FF, // Blue
    0xFFEA00, // Yellow
    0x00FF00, // Green
    0xFF0000, // Red
    0xCF067C, // Purple
    0xFF0800  // Orange
}; 


void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  // put your setup code here, to run once:
  RF_UPLINK_PORT.begin(RF_UPLINK_BAUD);
  RF_AV_DOWNLINK_PORT.begin(RF_AV_DOWNLINK_BAUD);
  RF_GSE_DOWNLINK_PORT.begin(RF_GSE_DOWNLINK_BAUD);

  UI_PORT.begin(115200);
  ROTATOR_PORT.begin(ROTATOR_BAUD);
  BINOCULARS_PORT.begin(BINOCULARS_BAUD);
  COMMAND_INPUT_PORT.begin(COMMAND_INPUT_BAUD);

  { 
    ledA.begin();
    ledA.fill(0x00FF00);
    ledA.show();
  }

  { 
    ledB.begin();
    ledB.fill(0x00FF00);
    ledB.show();
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  while (RF_UPLINK_PORT.available()) {
    RF_UPLINK.decode(RF_UPLINK_PORT.read());
  }

  while (RF_AV_DOWNLINK_PORT.available()) {
    RF_AV_DOWNLINK.decode(RF_AV_DOWNLINK_PORT.read());
  }

  while (RF_GSE_DOWNLINK_PORT.available()) {
    RF_GSE_DOWNLINK.decode(RF_GSE_DOWNLINK_PORT.read());
  }

  while (BINOCULARS_PORT.available()) {
    Binoculars.decode(BINOCULARS_PORT.read());
  }

  while (COMMAND_INPUT_PORT.available()) {
    CommandInput.decode(COMMAND_INPUT_PORT.read());
  }

  while (UI_PORT.available()) {
    Ui.decode(UI_PORT.read());
  } 

  if (rotator.isUpdated()) {
    rotator.setMode(rotator.computeMode());

    static rotatorCommand lastComputedCommand;
    rotatorCommand computedCommand = rotator.computeCommand();

    if ((lastComputedCommand.azm != computedCommand.azm) or (lastComputedCommand.elv != computedCommand.elv) or (lastComputedCommand.mode != computedCommand.mode)) {
      sendRotatorCmd(computedCommand);
      lastComputedCommand = computedCommand;
    }
  }
}

void sendRotatorCmd(rotatorCommand cmdToSend) {
  PacketTrackerCmd packetToSend;
  packetToSend.azm = cmdToSend.azm;
  packetToSend.elv = cmdToSend.elv;
  packetToSend.mode = cmdToSend.mode;

  UI_PORT.print("Sending command to rotator : ");
  UI_PORT.print(packetToSend.azm);
  UI_PORT.print(" ");
  UI_PORT.println(packetToSend.elv);

  byte* buffer = new byte[packetTrackerCmdSize]; // Allocate memory for the byte array
  memcpy(buffer, &packetToSend, packetTrackerCmdSize); // Copy the struct to the byte array

  uint8_t* bytesToSend = Rotator.encode(CAPSULE_ID::TRACKER_CMD,buffer,packetTrackerCmdSize);
  ROTATOR_PORT.write(bytesToSend,Rotator.getCodedLen(packetTrackerCmdSize));
  delete[] bytesToSend;
  delete[] buffer;
}

void handleRF_AV_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
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
  uint32_t ledColor = colors[random(0,7)];
  ledA.fill(ledColor);
  ledA.show();
}

void handleRF_GSE_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  // switch(packetId) {
  //   case CAPSULE_ID::GSE_TELEMETRY:
  //   {
  //     uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
  //     UI_PORT.write(packetToSend,Ui.getCodedLen(len));
  //     delete[] packetToSend;
  //   }
  //   break;
  //   default:
  //   break;
  // }
  
  uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
  UI_PORT.write(packetToSend,Ui.getCodedLen(len));
  delete[] packetToSend;

  uint32_t ledColor = colors[random(0,7)];
  ledA.fill(ledColor);
  ledA.show();
}

void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  uint8_t* packetToSend = RF_UPLINK.encode(packetId,dataIn,len);
  RF_UPLINK_PORT.write(packetToSend,RF_UPLINK.getCodedLen(len));
  delete[] packetToSend;
  switch (packetId) {
    case CAPSULE_ID::CALIBRATE_TELEMETRY:
      rotator.calibrateTelemetry();
    break;
    default:
    break;
  }
  uint32_t ledColor = colors[random(0,7)];
  ledB.fill(ledColor);
  ledB.show();
}

void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch(packetId) {
    case CAPSULE_ID::BINOC_ATTITUDE:
    {
      PacketBinocAttitude binocAttitude;
      memcpy(&binocAttitude, dataIn, packetBinocAttitudeSize);
      rotator.updateBinocAttitude(binocAttitude);
    }
    break;
    case CAPSULE_ID::BINOC_POSITION:
    {
      PacketBinocPosition binocPosition;
      memcpy(&binocPosition, dataIn, packetBinocPositionSize);
      rotator.updateBinocPosition(binocPosition);
    }
    break;
    case CAPSULE_ID::BINOC_STATUS:
    {
      PacketBinocStatus binocStatus;
      memcpy(&binocStatus, dataIn, packetBinocStatusSize);
      rotator.updateBinocStatus(binocStatus);
    }
    break;
    case CAPSULE_ID::BINOC_GLOBAL_STATUS:
    {
      PacketBinocGlobalStatus binocGlobalStatus;
      memcpy(&binocGlobalStatus, dataIn, packetBinocGlobalStatusSize);
      rotator.updateBinocGlobalStatus(binocGlobalStatus);
    }
    break;

    default:
    break;
  }
}

void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  //UI_PORT.println("Command received");
  switch(packetId) {
    case CAPSULE_ID::BINOC_ATTITUDE:
    {
      PacketBinocAttitude binocAttitude;
      memcpy(&binocAttitude, dataIn, packetBinocAttitudeSize);
      rotator.updateBinocAttitude(binocAttitude);
    }
    break;
    case CAPSULE_ID::BINOC_POSITION:
    {
      PacketBinocPosition binocPosition;
      memcpy(&binocPosition, dataIn, packetBinocPositionSize);
      rotator.updateBinocPosition(binocPosition);
    }
    break;
    case CAPSULE_ID::BINOC_STATUS:
    {
      PacketBinocStatus binocStatus;
      memcpy(&binocStatus, dataIn, packetBinocStatusSize);
      rotator.updateBinocStatus(binocStatus);
    }
    break;
    case CAPSULE_ID::BINOC_GLOBAL_STATUS:
    {
      PacketBinocGlobalStatus binocGlobalStatus;
      memcpy(&binocGlobalStatus, dataIn, packetBinocGlobalStatusSize);
      rotator.updateBinocGlobalStatus(binocGlobalStatus);
    }
    break;

    default:
    break;
  }
}

// These two little guys don't have anything to handle because the rotator and av up 
// are not sending anything back to the motherboard 

void handleRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}