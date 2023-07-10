#include <Arduino.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"

enum TRACKING_MODE {
    STATIONARY,
    TRACKING_BINOCULAR,
    TRACKING_TELEMETRY
};

struct rotatorCommand {
    float azm;
    float elv;
};

double azimuthTo(double lat1, double lng1, double lat2, double lng2);
double distanceTo(double lat1, double lng1, double lat2, double lng2);
rotatorCommand computeAngle(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2);

class rotClass {
  public:
    rotClass();
    void updateBinoc(PacketBinocGlobalStatus binocStatus);
    void updateBinocAttitude(PacketBinocAttitude binocAttitude);
    void updateBinocPosition(PacketBinocPosition binocPosition);
    void updateBinocStatus(PacketBinocStatus binocStatus);
    void updateBinocGlobalStatus(PacketBinocGlobalStatus binocStatus);
    void updateAV(PacketAV_downlink avStatus);
    bool isUpdated();
    void calibrateTelemetry();
    int  getTrackingRate();
    void setMode(TRACKING_MODE mode);
    TRACKING_MODE computeMode();
    TRACKING_MODE getMode();
    rotatorCommand getCommand();
    rotatorCommand computeCommand();
  private:
    TRACKING_MODE mode;
    PacketBinocGlobalStatus binoc;
    PacketAV_downlink av;
    rotatorCommand command;
    bool rocketPositionIsUpdated;
    bool binocAttitudeIsUpdated;
    bool binocPositionIsUpdated;
    bool binocStatusIsUpdated;
    bool binocGlobalStatusIsUpdated;
    bool telemetryCalibrated;
    rotatorCommand tlmOffset;
    int trackingRate;
};
