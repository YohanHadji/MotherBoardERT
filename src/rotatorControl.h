#include <Arduino.h>

struct rotatorCommand {
    double azm;
    double elv;
};

double azimuthTo(double lat1, double lng1, double lat2, double lng2);
double distanceTo(double lat1, double lng1, double lat2, double lng2);

rotatorCommand computeRotatorCommand(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2) {
    rotatorCommand command;
    command.azm = azimuthTo(lat1, lng1, lat2, lng2);
    command.elv = (atan((alt2 - alt1) / distanceTo(lat1, lng1, lat2, lng2))/PI)*180;
    return command;
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
