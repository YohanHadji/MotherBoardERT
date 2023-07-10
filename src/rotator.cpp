#include "rotator.h"

rotClass::rotClass() {

};

bool rotClass::isUpdated() {
    switch(mode) {
        case STATIONARY:
            return true;
        break;
        case TRACKING_BINOCULAR:
            if (binocAttitudeIsUpdated) {
            binocAttitudeIsUpdated = false;
            return true;
            }
            if (binocPositionIsUpdated) {
                binocPositionIsUpdated = false;
                return true;
            }
            if (binocStatusIsUpdated) {
                binocStatusIsUpdated = false;
                return true;
            }
            if (binocGlobalStatusIsUpdated) {
                binocGlobalStatusIsUpdated = false;
                return true;
            }
        break;
        case TRACKING_TELEMETRY:
            if (rocketPositionIsUpdated) {
                rocketPositionIsUpdated = false;
                return true;
            }
        break;
    } 
    return false;     
};

void rotClass::updateAV(PacketAV_downlink avStatus) {
    av = avStatus;
    rocketPositionIsUpdated = true;
};

void rotClass::updateBinocAttitude(PacketBinocAttitude binocAttitude) {
    binoc.attitude = binocAttitude;
    binocAttitudeIsUpdated = true;
};

void rotClass::updateBinocPosition(PacketBinocPosition binocPosition) {
    binoc.position = binocPosition;
    binocPositionIsUpdated = true;
};

void rotClass::updateBinocStatus(PacketBinocStatus binocStatus) {
    binoc.status = binocStatus;
    binocStatusIsUpdated = true;
};

void rotClass::updateBinocGlobalStatus(PacketBinocGlobalStatus binocStatus) {
    binoc = binocStatus;
    binocGlobalStatusIsUpdated = true;
};

rotatorCommand rotClass::getCommand() {
    return command;
};

TRACKING_MODE rotClass::getMode() {
    return mode;
};

void rotClass::setMode(TRACKING_MODE modeIn) {
    mode = modeIn;
};

rotatorCommand rotClass::computeCommand() {
    rotatorCommand commandOut;
    mode = computeMode();
    switch (mode) {
        case STATIONARY:
            commandOut.azm = 0;
            commandOut.elv = 0;
        break;
        case TRACKING_BINOCULAR:
            commandOut.azm = binoc.attitude.azm;
            commandOut.elv = binoc.attitude.elv;
        break;
        case TRACKING_TELEMETRY:
            rotatorCommand avRaw;
            avRaw = computeAngle(binoc.position.lat, binoc.position.lon, binoc.position.alt, av.gnss_lat, av.gnss_lon, av.gnss_alt);
            commandOut.azm = avRaw.azm - tlmOffset.azm;
            commandOut.elv = avRaw.elv - tlmOffset.elv;
        break;
    } 
    return commandOut;
};

TRACKING_MODE rotClass::computeMode() {
    if (binoc.status.isInView and binoc.status.isCalibrated) {
        return TRACKING_BINOCULAR;
    } 
    else if (telemetryCalibrated) {
        return TRACKING_TELEMETRY;
    }
    else {
        return STATIONARY;
    }
}

void rotClass::calibrateTelemetry() {
   rotatorCommand avRaw;
    avRaw = computeAngle(binoc.position.lat, binoc.position.lon, binoc.position.alt, av.gnss_lat, av.gnss_lon, av.gnss_alt);
    tlmOffset.azm = avRaw.azm;
    tlmOffset.elv = avRaw.elv;
}

double azimuthTo(double lat1, double lng1, double lat2, double lng2) {
  lat1 = lat1 * PI / 180;
  lng1 = lng1 * PI / 180;
  lat2 = lat2 * PI / 180;
  lng2 = lng2 * PI / 180;

  double dlon = lng2-lng1;
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += 2*PI;
  }
  return a2/PI*180.0;
}

double distanceTo(double lat1, double lng1, double lat2, double lng2) {
  double R = 6371000;
  lat1 = lat1 * PI / 180.0;
  lng1 = lng1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  lng2 = lng2 * PI / 180.0;

  double dlat = lat2-lat1;
  double dlng = lng2-lng1;

  double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlng/2) * sin(dlng/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c;
  return d;
}

rotatorCommand computeAngle(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2) {
    rotatorCommand command;
    command.azm = azimuthTo(lat1, lng1, lat2, lng2);
    command.elv = (atan((alt2 - alt1) / distanceTo(lat1, lng1, lat2, lng2))/PI)*180;
    return command;
}


