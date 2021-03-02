#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
 
void cloudCB(const sensor_msgs::PointCloud2 &input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息
  pcl::io::savePCDFileASCII ("/home/allen/catkin_ws/src/my_pcl_tutorial/data/pcd_50610.pcd", cloud);//保存pcd
}
main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_save");
  ros::NodeHandle nh;
  ros::Subscriber bat_sub = nh.subscribe("pcl_output", 10, cloudCB);//接收点云
  //ros::Subscriber bat_sub = nh.subscribe("camera/depth/points", 10, cloudCB);//接收点云pcl_output
  ros::spin();
  return 0;
}
