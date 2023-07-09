#include <Arduino.h>

enum CAPSULE_ID {
    AIR_TO_GROUND = 0x00,
    GROUND_TO_AIR,
    MOTHER_TO_DEVICE,
    DEVICE_TO_MOTHER,
    PC_TO_MOTHER
};

struct telemetryPacket {
  uint32_t timeSecond;

  uint8_t  flightMode;
  uint8_t  status;

  float_t  latitude;
  float_t  longitude;
  float_t  altitude;

  float_t  yaw;

  float_t  zSpeed;
  float_t  twoDSpeed;

  float_t  temperature;
  float_t  voltage;

  float_t  waypointLatitude;
  float_t  waypointLongitude;
  float_t  waypointAltitude;

  float_t  trajectoryLatitude;
  float_t  trajectoryLongitude;

  float_t  distanceToTrajectory;
  float_t  distanceToPosition;
};

telemetryPacket lastPacket;

