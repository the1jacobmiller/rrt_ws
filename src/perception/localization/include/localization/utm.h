#ifndef UTM_H
#define UTM_H

using namespace std;

#include <ros/ros.h>
#include <cmath>
#include <string>

const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

#define WGS84_A 6378137.0 // major axis
#define WGS84_E 0.0818191908 // first eccentricity
#define UTM_E2 (WGS84_E*WGS84_E) // e^2
#define UTM_K0 0.9996 // scale factor

float calcOffsetAngle(long double lat, long double lon, int zone_num);
char UTMLetterDesignator(double Lat);
void LLtoUTM(long double Lat, long double Long, long double &UTMNorthing, long double &UTMEasting, std::string &UTMZone, double &gamma);
void LLtoSpecifiedZoneUTM(long double Lat, long double Long, int ZoneNumber, long double &UTMNorthing, long double &UTMEasting, std::string &UTMZone, double &gamma);
void checkUTMZones(bool &load_two_zones, int &adjacent_UTM_zone, long double lat_degree, long double lon_degree, int current_UTM_zone);

#endif
