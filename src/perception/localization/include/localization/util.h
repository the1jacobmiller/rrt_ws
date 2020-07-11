#ifndef __UTIL_H__
#define __UTIL_H__

#include <ros/ros.h>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>

#include "localization/point.h"

void toEulerAngle(const geometry_msgs::Quaternion &q, double &yaw, double &pitch, double &roll);
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);
double distance2DPoints(point point1, point point2);
double getAngleToPoint(point point1, point point2);
double getAngleDiff(double angle1, double angle2);
double ms2mph(double speed_ms);
double mph2ms(double speed_mph);
point findClosestProjectedPoint(point cp1, point cp2, point target_point);

#endif
