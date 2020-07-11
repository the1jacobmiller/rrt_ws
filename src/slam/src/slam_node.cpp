#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {

}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle n;
  ros::Rate r(30);

  // Publishers


  // Subscribers
  ros::Subscriber sub_lidar = n.subscribe("/points_raw", 1, lidarCallback);

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();
}
