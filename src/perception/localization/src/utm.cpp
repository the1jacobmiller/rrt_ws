#include "localization/utm.h"

float calcOffsetAngle(long double lat, long double lon, int zone_num) {
  float center_degree_offset;
  if (!ros::param::get("/center_degree_offset", center_degree_offset)) {
    ROS_ERROR_STREAM("Pub GNSS Pose - Failed to read OxTS offset angle");
  }

  float lon_origin = (zone_num - 1)*6.0 - 180 + 6.0/2.0;
  float degrees_lon_from_center = lon - lon_origin;
  float offset_angle = 0.66 * degrees_lon_from_center + center_degree_offset;

  offset_angle = offset_angle * (M_PI / 180.0); // convert to radians
  return offset_angle;
}

char UTMLetterDesignator(double Lat) {
  char LetterDesignator;

  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = 'Z';
  return LetterDesignator;
}

void LLtoUTM(long double Lat, long double Long, long double &UTMNorthing, long double &UTMEasting, std::string &UTMZone, double &gamma) {
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-static_cast<int>((Long+180)/360)*360-180;

  double LatRad = Lat*RADIANS_PER_DEGREE;
  double LongRad = LongTemp*RADIANS_PER_DEGREE;
  double LongOriginRad;
  int    ZoneNumber;

  ZoneNumber = static_cast<int>((LongTemp + 180)/6.0) + 1;

  if ( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    ZoneNumber = 32;

  // Special zones for Svalbard
  if ( Lat >= 72.0 && Lat < 84.0 )
  {
    if (      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
    else if ( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
    else if ( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
    else if ( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
  }
        // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6.0 - 180 + 6.0/2.0;
  LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

  // Compute the UTM Zone from the latitude and longitude
  char zone_buf[] = {0, 0, 0, 0};
  snprintf(zone_buf, sizeof(zone_buf), "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  UTMZone = std::string(zone_buf);

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);

  M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
               - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
               + (15*eccSquared*eccSquared/256
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

  UTMEasting = static_cast<long double>
          (k0*N*(A+(1-T+C)*A*A*A/6
                 + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
           + 500000.0);

  UTMNorthing = static_cast<long double>
          (k0*(M+N*tan(LatRad)
               *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

  gamma = atan(tan(LongRad-LongOriginRad)*sin(LatRad)) * DEGREES_PER_RADIAN;

  if (Lat < 0)
          {
            // 10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0;
          }
}

void LLtoSpecifiedZoneUTM(long double Lat, long double Long, int ZoneNumber, long double &UTMNorthing, long double &UTMEasting, std::string &UTMZone, double &gamma) {
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-static_cast<int>((Long+180)/360)*360-180;

  double LatRad = Lat*RADIANS_PER_DEGREE;
  double LongRad = LongTemp*RADIANS_PER_DEGREE;
  double LongOriginRad;

  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6.0 - 180 + 6.0/2.0;
  LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

  // Compute the UTM Zone from the latitude and longitude
  char zone_buf[] = {0, 0, 0, 0};
  snprintf(zone_buf, sizeof(zone_buf), "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  UTMZone = std::string(zone_buf);

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);

  M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
               - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
               + (15*eccSquared*eccSquared/256
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

  UTMEasting = static_cast<long double>
          (k0*N*(A+(1-T+C)*A*A*A/6
                 + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
           + 500000.0);

  UTMNorthing = static_cast<long double>
          (k0*(M+N*tan(LatRad)
               *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

  gamma = atan(tan(LongRad-LongOriginRad)*sin(LatRad)) * DEGREES_PER_RADIAN;

  if (Lat < 0)
          {
            // 10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0;
          }
}

void checkUTMZones(bool &load_two_zones, int &adjacent_UTM_zone, long double lat_degree, long double lon_degree, int current_UTM_zone) {
  long double UTMNorthing, UTMEasting;
  double gamma;
  string UTMZone;
  string::size_type sz;

  long double degrees_per_tile = 0.02197265625;
  long double larger_lon = lon_degree + 2 * degrees_per_tile;
  LLtoUTM(lat_degree, larger_lon, UTMNorthing, UTMEasting, UTMZone, gamma);
  int zone_num = stoi(UTMZone.substr(0, UTMZone.length()-1), &sz);
  if (zone_num != current_UTM_zone) {
    ROS_INFO_STREAM("UTM - Loading two UTM zones");
    load_two_zones = true;
    adjacent_UTM_zone = zone_num;
    return;
  }

  long double smaller_lon = lon_degree - 2 * degrees_per_tile;
  LLtoUTM(lat_degree, smaller_lon, UTMNorthing, UTMEasting, UTMZone, gamma);
  zone_num = stoi(UTMZone.substr(0, UTMZone.length()-1), &sz);
  if (zone_num != current_UTM_zone) {
    ROS_INFO_STREAM("UTM - Loading two UTM zones");
    load_two_zones = true;
    adjacent_UTM_zone = zone_num;
    return;
  }
}
