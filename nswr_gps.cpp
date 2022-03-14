#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nswr_gps/conversions.h>

static ros::Publisher odom_tapas_pub, odom_magellan_pub;
double dist_tapas = 0, dist_magellan=0;

void callback_tapas(const sensor_msgs::NavSatFixConstPtr &fix)
{
  if (fix->header.stamp == ros::Time(0) || fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    return;

  double northing, easting;
  std::string zone;

  // Convert latitude and longitude to UTM coordinates 
  gps_common::LLtoUTM(fix->latitude ,fix->longitude, northing, easting, zone);

  // Declare odometry message
  nav_msgs::Odometry odom;

  // Fill odometry message time and frames
  odom.header.stamp = fix->header.stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = fix->header.frame_id;

  // Fill "position" in odom message (http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
  // easting: the x coordinate
  // northing: the y coordinate
  // fix->altitude: the z coordinate
  static bool firstPoint = true;
  static double x0, y0, z0;
  static double x_prev, y_prev, z_prev;
  if(firstPoint)
  {
      x0 = easting;
      y0 = northing;
      z0 = fix->altitude;
      firstPoint = false;
      //std::cout << "ok " << x0 << '/' << y0 << '/' << z0 << std::endl;
  }
  odom.pose.pose.position.x = easting - x0;
  odom.pose.pose.position.y = northing - y0;
  odom.pose.pose.position.z = fix->altitude - z0;
  //std::cout << odom.pose.pose.position.x << '/' << odom.pose.pose.position.y << '/' << odom.pose.pose.position.z << std::endl;
  bool static flag2 = true;
  if(flag2)
  {
          flag2 = false;
  }
  else
  {
      double x = pow((odom.pose.pose.position.x - x_prev), 2);
      double y = pow((odom.pose.pose.position.y - y_prev), 2);
      double z = pow((odom.pose.pose.position.z - z_prev), 2);
      dist_tapas += sqrt(x+y+z);
      //std::cout << "tapas distance: " << dist_tapas << std::endl;
  }
  x_prev = odom.pose.pose.position.x;
  y_prev = odom.pose.pose.position.y;
  z_prev = odom.pose.pose.position.z;
  // Publish odometry message
  odom_tapas_pub.publish(odom);
}

void callback_magellan(const sensor_msgs::NavSatFixConstPtr &fix)
{
  if (fix->header.stamp == ros::Time(0) || fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    return;

  double northing, easting;
  std::string zone;

  // Convert latitude and longitude to UTM coordinates
  gps_common::LLtoUTM(fix->latitude ,fix->longitude, northing, easting, zone);

  // Declare odometry message
  nav_msgs::Odometry odom;

  // Fill odometry message time and frames
  odom.header.stamp = fix->header.stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = fix->header.frame_id;

  // Fill "position" in odom message (http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
  // easting: the x coordinate
  // northing: the y coordinate
  // fix->altitude: the z coordinate
  static bool firstPoint = true;
  static double x0, y0, z0;
  static double x_prev, y_prev, z_prev;
  if(firstPoint)
  {
      x0 = easting;
      y0 = northing;
      z0 = fix->altitude;
      firstPoint = false;
      //std::cout << "ok " << x0 << '/' << y0 << '/' << z0 << std::endl;
  }
  odom.pose.pose.position.x = easting - x0;
  odom.pose.pose.position.y = northing - y0;
  odom.pose.pose.position.z = fix->altitude - z0;
  //std::cout << odom.pose.pose.position.x << '/' << odom.pose.pose.position.y << '/' << odom.pose.pose.position.z << std::endl;
  bool static flag2 = true;
  if(flag2)
  {
          flag2 = false;
  }
  else
  {
      double x = pow((odom.pose.pose.position.x - x_prev), 2);
      double y = pow((odom.pose.pose.position.y - y_prev), 2);
      double z = pow((odom.pose.pose.position.z - z_prev), 2);
      dist_magellan += sqrt(x+y+z);
      //std::cout << "magellan distance: " << dist_magellan << std::endl;
  }
  x_prev = odom.pose.pose.position.x;
  y_prev = odom.pose.pose.position.y;
  z_prev = odom.pose.pose.position.z;

  // Publish odometry message
  odom_magellan_pub.publish(odom);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nswr_gps");
  ros::NodeHandle node("~");

  // Odometry publishers
  odom_tapas_pub = node.advertise<nav_msgs::Odometry>("odom_tapas", 10);
  odom_magellan_pub = node.advertise<nav_msgs::Odometry>("odom_magellan", 10);

  // GPS Fix subscribers
  ros::Subscriber fix_tapas_sub = node.subscribe("/gps_tapas/fix", 10, callback_tapas);
  ros::Subscriber fix_magellan_sub = node.subscribe("/dgps_magellan/fix", 10, callback_magellan);

  ros::spin();
  std::cout << "tapas distance: " << dist_tapas << std::endl;
  std::cout << "magellan distance: " << dist_magellan << std::endl;
}
