#include "localization/util.h"
#include "localization/utm.h"

#include <ros/ros.h>
#include <string>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <gps_common/GPSFix.h>

using namespace std;

ros::Publisher gnss_pose_pub;

void publishTF(geometry_msgs::PoseStamped gnss_pose) {
  static tf::TransformBroadcaster br_;

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(gnss_pose.pose.position.x, gnss_pose.pose.position.y, 0.0));
  tf::Quaternion quaternion = tf::Quaternion(gnss_pose.pose.orientation.x, gnss_pose.pose.orientation.y, gnss_pose.pose.orientation.z, gnss_pose.pose.orientation.w);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "gps"));
}

void simulatedGPSCallback(const gps_common::GPSFix::ConstPtr msg) {
  float center_degree_offset;
  if (!ros::param::get("/center_degree_offset", center_degree_offset)) {
    ROS_ERROR_STREAM("Pub GNSS Pose - Failed to read OxTS offset angle");
  }

  long double lon = msg->longitude;
  long double lat = msg->latitude;

  long double UTMNorthing, UTMEasting;
  double gamma;
  string UTMZone;
  LLtoUTM(lat, lon, UTMNorthing, UTMEasting, UTMZone, gamma);

  string::size_type sz;
  int zone_num = stoi(UTMZone.substr(0, UTMZone.length()-1), &sz);

  float offset_angle = calcOffsetAngle(lat, lon, zone_num);

  double track = msg->track - 90.0;
  if (track >= 0.0 && track <= 180.0)
    track= track * -1.0;
  else track = 360.0 - track;
  double yaw=track*M_PI/180.0;
  yaw = yaw + offset_angle;
  if (yaw > M_PI) yaw -= 2*M_PI;

  // ROS_INFO_STREAM("Pub GNSS - lat: " << lat << ", lon: " << lon);
  // ROS_INFO_STREAM("Pub GNSS - angle offset: " << offset_angle);

  // ROS_INFO_STREAM("Pub GNSS - Northing: " << UTMNorthing << ", Easting: " << UTMEasting << ", UTM Zone: " << UTMZone);

  geometry_msgs::PoseStamped gnss_pose;
  gnss_pose.pose.position.x = UTMEasting;
  gnss_pose.pose.position.y = UTMNorthing;
  gnss_pose.pose.orientation = toQuaternion(0.0, 0.0, yaw);
  gnss_pose_pub.publish(gnss_pose);
  publishTF(gnss_pose);
}

void GPSCallback(const visualization_msgs::Marker::ConstPtr msg) {
  float center_degree_offset;
  if (!ros::param::get("/center_degree_offset", center_degree_offset)) {
    ROS_ERROR_STREAM("Pub GNSS Pose - Failed to read OxTS offset angle");
  }

  long double lon = (msg->pose.position.x * 180.0) / M_PI;
  long double lat = (msg->pose.position.y * 180.0) / M_PI;

  long double UTMNorthing, UTMEasting;
  double gamma;
  string UTMZone;
  LLtoUTM(lat, lon, UTMNorthing, UTMEasting, UTMZone, gamma);

  string::size_type sz;
  int zone_num = stoi(UTMZone.substr(0, UTMZone.length()-1), &sz);

  float offset_angle = calcOffsetAngle(lat, lon, zone_num);

  double yaw, pitch, roll;
  toEulerAngle(msg->pose.orientation, yaw, pitch, roll);
  yaw = yaw + offset_angle;


  // ROS_INFO_STREAM("Pub GNSS - lat: " << lat << ", lon: " << lon);
  // ROS_INFO_STREAM("Pub GNSS - angle offset: " << offset_angle);

  // ROS_INFO_STREAM("Pub GNSS - Northing: " << UTMNorthing << ", Easting: " << UTMEasting << ", UTM Zone: " << UTMZone);

  geometry_msgs::PoseStamped gnss_pose;
  gnss_pose.pose.position.x = UTMEasting;
  gnss_pose.pose.position.y = UTMNorthing;
  gnss_pose.pose.orientation = toQuaternion(pitch, roll, yaw);
  gnss_pose_pub.publish(gnss_pose);
  publishTF(gnss_pose);
}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "pub_gnss_pose");
  ros::NodeHandle n;
  ros::Rate r(30);

  bool simulation_mode;
  if (!ros::param::get("/simulation_mode", simulation_mode) ) {
    ROS_ERROR_STREAM("Failed to read ROS configs in main - pub gnss pose");
    ros::shutdown();
  }

  // Publishers
  gnss_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 1);

  // Subscribers
  ros::Subscriber sub_gps;
  if (simulation_mode) {
    ROS_WARN_STREAM("Pub GNSS - Running in simulation mode!");
    sub_gps = n.subscribe("/fusion/perfect_gps/enhanced_fix", 1, simulatedGPSCallback);
  } else {
    sub_gps = n.subscribe("/gps", 1, GPSCallback);
  }

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();
}
