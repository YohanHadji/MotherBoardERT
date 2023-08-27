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
void handleAntennaRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleCameraRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len);

void sendRotatorCmd(PacketTrackerCmd cmd);

Adafruit_NeoPixel ledA(1, NEOPIXEL_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
Adafruit_NeoPixel ledB(1, NEOPIXEL_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

CapsuleStatic RF_UPLINK(handleRF_UPLINK);
CapsuleStatic RF_AV_DOWNLINK(handleRF_AV_DOWNLINK);
CapsuleStatic RF_GSE_DOWNLINK(handleRF_GSE_DOWNLINK);
CapsuleStatic Ui(handleUi);
CapsuleStatic AntennaRotator(handleAntennaRotator);
CapsuleStatic CameraRotator(handleCameraRotator);
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
  ANTENNA_ROTATOR_PORT.begin(ANTENNA_ROTATOR_BAUD);
  CAMERA_ROTATOR_PORT.begin(CAMERA_ROTATOR_BAUD);
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
  rotator.begin();

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
    static TARGET_MODE lastMode = TARGET_MODE::TARGET_NONE;

    rotator.setMode(rotator.computeMode());

    if (rotator.getMode() != lastMode) {
      if (lastMode == TARGET_MODE::TARGET_NONE and rotator.getMode() == TARGET_MODE::TARGET_POSITION) {
        position lastPosition = rotator.getPosition();
        rotator.latEstimator.reset(lastPosition.lat);
        rotator.lonEstimator.reset(lastPosition.lon);
        rotator.altEstimator.reset(lastPosition.alt);
      }
      lastMode = rotator.getMode();
    }

    // UI_PORT.print("Rotator mode : ");
    // UI_PORT.println(rotator.getMode());

    static rotatorCommand lastComputedCommand;
    rotatorCommand computedCommand = rotator.computeCommand();

    PacketTrackerCmd cmdToSend;

    if ((lastComputedCommand.azm != computedCommand.azm) or (lastComputedCommand.elv != computedCommand.elv) or (lastComputedCommand.mode != computedCommand.mode)) {
      cmdToSend.azm = computedCommand.azm;
      cmdToSend.elv = computedCommand.elv;
      cmdToSend.mode = computedCommand.mode;

      switch (rotator.getMode()) {
        case TARGET_MODE::TARGET_NONE:
        case TARGET_MODE::TARGET_POINTER:
          cmdToSend.timeStamp = millis();
          cmdToSend.cutoffFreq = 20;
          cmdToSend.maxTimeWindow = (1000.0/20.0)*5.0;
        break;
        case TARGET_MODE::TARGET_POSITION:
          cmdToSend.timeStamp = 0; // rotator.getAvTime();
          cmdToSend.cutoffFreq = 0.1;
          cmdToSend.maxTimeWindow = (1000.0/1.0)*5.0;
        break;
        case TARGET_MODE::TARGET_VISION:
        break;
      }
      sendRotatorCmd(cmdToSend);
      lastComputedCommand = computedCommand;
    }
  }
}

void sendRotatorCmd(PacketTrackerCmd packetToSend) {

  // UI_PORT.print("Sending command to rotator : ");
  // UI_PORT.print(packetToSend.azm);
  // UI_PORT.print(" ");
  // UI_PORT.println(packetToSend.elv);

  byte* buffer = new byte[packetTrackerCmdSize]; // Allocate memory for the byte array
  memcpy(buffer, &packetToSend, packetTrackerCmdSize); // Copy the struct to the byte array

  uint8_t* bytesToSendAntenna = AntennaRotator.encode(CAPSULE_ID::TRACKER_CMD,buffer,packetTrackerCmdSize);
  ANTENNA_ROTATOR_PORT.write(bytesToSendAntenna,AntennaRotator.getCodedLen(packetTrackerCmdSize));

  uint8_t* bytesToSendCamera = CameraRotator.encode(CAPSULE_ID::TRACKER_CMD,buffer,packetTrackerCmdSize);
  CAMERA_ROTATOR_PORT.write(bytesToSendCamera,CameraRotator.getCodedLen(packetTrackerCmdSize));

  delete[] bytesToSendAntenna;
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

      av_downlink_t lastPacket;
      memcpy(&lastPacket, dataIn, av_downlink_size);

      position lastAvPosition;

      lastAvPosition.lat = lastPacket.gnss_lat;
      lastAvPosition.lon = lastPacket.gnss_lon;
      lastAvPosition.alt = lastPacket.gnss_alt;

      rotator.updatePosition(lastAvPosition);

      rotator.latEstimator.update(lastPacket.gnss_lat,millis()-lastPacket.timestamp);
      rotator.lonEstimator.update(lastPacket.gnss_lon,millis()-lastPacket.timestamp);
      rotator.altEstimator.update(lastPacket.gnss_alt,millis()-lastPacket.timestamp);

      uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
      // UI_PORT.write(packetToSend,Ui.getCodedLen(len));
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
      // Do something??
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
    case CAPSULE_ID::BINOC_GLOBAL_STATUS:
    {
      PacketBinocGlobalStatus binocGlobalStatus;
      memcpy(&binocGlobalStatus, dataIn, packetBinocGlobalStatusSize);

      pointer lastBinocPointer;
      lastBinocPointer.azm = binocGlobalStatus.attitude.azm;
      lastBinocPointer.elv = binocGlobalStatus.attitude.elv;
      lastBinocPointer.isInView = binocGlobalStatus.status.isInView;
      lastBinocPointer.isCalibrated = binocGlobalStatus.status.isCalibrated;

      rotator.updatePointer(lastBinocPointer);
    }
    break;

    default:
    break;
  }
}

void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  //UI_PORT.println("Command received");
  switch(packetId) {
    case CAPSULE_ID::BINOC_GLOBAL_STATUS:
    {
      PacketBinocGlobalStatus binocGlobalStatus;
      memcpy(&binocGlobalStatus, dataIn, packetBinocGlobalStatusSize);

      pointer lastBinocPointer;
      
      lastBinocPointer.azm = binocGlobalStatus.attitude.azm;
      lastBinocPointer.elv = binocGlobalStatus.attitude.elv;
      lastBinocPointer.isInView = binocGlobalStatus.status.isInView;
      lastBinocPointer.isCalibrated = binocGlobalStatus.status.isCalibrated;

      rotator.updatePointer(lastBinocPointer);
    }
    break;

    default:
    break;
  }
}

// These two little guys don't have anything to handle because the rotator and av up 
// are not sending anything back to the motherboard 

void handleCameraRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}

void handleAntennaRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}