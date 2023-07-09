#include <Arduino.h>
#include <Capsule.h>  
#include <Adafruit_NeoPixel.h>
#include "packetInterface.h"
#include "rotatorControl.h"

#define CMD_MANUAL_MODE false

#define RF1_PORT Serial8
#define RF1_BAUD 115200

#define RF2_PORT Serial7
#define RF2_BAUD 115200

#define RF3_PORT Serial6
#define RF3_BAUD 115200

#define ROTATOR_PORT Serial1
#define ROTATOR_BAUD 19200

#define NEOPIXEL_A_PIN 33
#define NEOPIXEL_B_PIN 32

void handleRF1(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF2(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF3(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleMac(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel ledA(1, NEOPIXEL_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
Adafruit_NeoPixel ledB(1, NEOPIXEL_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

CapsuleStatic RF1(handleRF1);
CapsuleStatic RF2(handleRF2);
CapsuleStatic RF3(handleRF3);
CapsuleStatic Mac(handleMac);

String sendToMac();

static bool positionIsUpdated = false;

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
  RF1_PORT.begin(RF1_BAUD);
  RF2_PORT.begin(RF2_BAUD);

  Serial.begin(115200);
  ROTATOR_PORT.begin(ROTATOR_BAUD);
  // Serial.println("Starting...");

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
  while (RF1_PORT.available()) {
    RF1.decode(RF1_PORT.read());
  }

  while (RF2_PORT.available()) {
    RF2.decode(RF2_PORT.read());
  }

  while (RF3_PORT.available()) {
    RF3.decode(RF3_PORT.read());
  }

  if (CMD_MANUAL_MODE) {
    const int cmdBufferSize = 256;
    static uint8_t buff[cmdBufferSize];
    static int buffIndex = 0;

    while(Serial.available()) {
      uint8_t a = Serial.read();
      if (a == '#') {
        for (int i = 0; i < cmdBufferSize; i++) {
          buff[i] = 0;
        }
      }
      else if (a != '/') {
        buff[buffIndex++] = a;
      }
      else {
        // Send packet to radiomodule 1 (Only the radiomodule 1 is transmitting stuff)
        size_t codedSize = RF1.getCodedLen(buffIndex);
        byte* coded = new byte[codedSize];
        coded = RF1.encode(CAPSULE_ID::MOTHER_TO_DEVICE, buff, buffIndex);

        RF1_PORT.write(coded, codedSize);
        buffIndex = 0;
        delete[] coded;
        for (int i = 0; i < cmdBufferSize; i++) {
          buff[i] = 0;
        }
      }
    }
  }
  else {
    while (Serial.available()) {
      Mac.decode(Serial.read());
    } 
  }

  if (positionIsUpdated) {
    positionIsUpdated = false;
    double payerneLat = 46.8130919;
    double payerneLon = 6.9435166;
    double payerneAlt = 540.5;
    double currentLat = lastPacket.latitude;
    double currentLon = lastPacket.longitude;
    double currentAlt = lastPacket.altitude;
    rotatorCommand newCmd;
    newCmd = computeRotatorCommand(payerneLat, payerneLon, payerneAlt, currentLat, currentLon, currentAlt);
    String newOutput;
    newOutput = "";
    newOutput += "AZ";
    newOutput += String(newCmd.azm);
    newOutput += " EL";
    newOutput += String(newCmd.elv);
    ROTATOR_PORT.println(newOutput);
  }
}

void handleRF1(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::DEVICE_TO_MOTHER:
    {
      // Serial.println("Packet with ID 00 from RF1 received : ");
      uint32_t ledColor = colors[random(sizeof(colors)/sizeof(uint32_t))];
      ledA.fill(ledColor);
      ledA.show();

      memcpy(&lastPacket, dataIn, sizeof(lastPacket));
      String output = sendToMac();
      Serial.println("#"+sendToMac()+"/");

      positionIsUpdated = true;
      //saveOnSd(output);
      //printOnScreen();
      //digitalWrite(BOARD_LED, !LED_ON);
    }
    break;
    default:
    break;
  }
}

void handleRF2(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::DEVICE_TO_MOTHER:
    {
      // Serial.println("Packet with ID 00 from RF2 received : ");
      uint32_t ledColor = colors[random(sizeof(colors)/sizeof(uint32_t))];
      ledB.fill(ledColor);
      ledB.show();

      memcpy(&lastPacket, dataIn, sizeof(lastPacket));
      String output = sendToMac();
      Serial.println("#"+sendToMac()+"/");

      positionIsUpdated = true;
      //saveOnSd(output);
      //printOnScreen();
      //digitalWrite(BOARD_LED, !LED_ON);
    }
    break;
    default:
    break;
  }
}

void handleRF3(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::DEVICE_TO_MOTHER:
    {
      // Serial.println("Packet with ID 00 from RF2 received : ");
      uint32_t ledColor = colors[random(sizeof(colors)/sizeof(uint32_t))];
      ledB.fill(ledColor);
      ledB.show();

      memcpy(&lastPacket, dataIn, sizeof(lastPacket));
      String output = sendToMac();
      Serial.println("#"+sendToMac()+"/");

      positionIsUpdated = true;
      //saveOnSd(output);
      //printOnScreen();
      //digitalWrite(BOARD_LED, !LED_ON);
    }
    break;
    default:
    break;
  }
}

void handleMac(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::PC_TO_MOTHER:
    {
      // Serial.println("Packet with ID 00 from MAC received : ");
      uint8_t* packetToSend = RF1.encode(CAPSULE_ID::MOTHER_TO_DEVICE,dataIn,len);
      RF1_PORT.write(packetToSend,RF1.getCodedLen(len));
      delete[] packetToSend;
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
    break;
    default:
    break;
  }
}


String sendToMac() {
  String output = "";
  output+= String(lastPacket.timeSecond)+",";
  output+= String(lastPacket.flightMode)+",";

  // packet.status is 1 byte where each bit represents a status. 
  // Bit 0 = initialised
  // Bit 1 = separated
  // Bit 2 = deployed 
  // Bit 3 = wing opened 
  // Bit 4 = sensor valid 
  // Bit 5 = gps valid
  // Bit 6 = time valid

  output+= String((lastPacket.status & 0x01) ? 1 : 0)+","; // initialised
  output+= String((lastPacket.status & 0x02) ? 1 : 0)+","; // separated
  output+= String((lastPacket.status & 0x04) ? 1 : 0)+","; // deployed
  output+= String((lastPacket.status & 0x08) ? 1 : 0)+","; // wing opened
  output+= String((lastPacket.status & 0x10) ? 1 : 0)+","; // sensor valid
  output+= String((lastPacket.status & 0x20) ? 1 : 0)+","; // gps valid
  output+= String((lastPacket.status & 0x40) ? 1 : 0)+","; // time valid


  output+= String(lastPacket.latitude,7)+",";
  output+= String(lastPacket.longitude,7)+",";
  output+= String(lastPacket.altitude,2)+",";
  output+= String(lastPacket.yaw,1)+",";
  output+= String(lastPacket.zSpeed,2)+",";
  output+= String(lastPacket.twoDSpeed,2)+",";
  output+= String(lastPacket.temperature,0)+",";
  output+= String(lastPacket.voltage,2)+",";
  output+= String(lastPacket.waypointLatitude,6)+",";
  output+= String(lastPacket.waypointLongitude,6)+",";
  output+= String(lastPacket.waypointAltitude,0)+",";
  output+= String(lastPacket.trajectoryLatitude,6)+",";
  output+= String(lastPacket.trajectoryLongitude,6)+",";
  output+= String(lastPacket.distanceToTrajectory,0)+",";
  output+= String(lastPacket.distanceToPosition,0);
  output+= "";

  return output;
}