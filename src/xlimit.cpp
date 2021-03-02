#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//滤波的头文件
#include <pcl/filters/passthrough.h>
//申明发布器
ros::Publisher pub;
//回调函数

void//zhitong
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // build the filter
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
    // apply filter
  pass.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_pt;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_pt);

  // Publish the data
  pub.publish (cloud_pt);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "xlimit");//声明节点的名称
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
// 为接受点云数据创建一个订阅节点
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("downsampled", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
//创建ROS的发布节点
  pub = nh.advertise<sensor_msgs::PointCloud2> ("xlimited", 1);
//    ros::Duration(0.05).sleep();
// 回调
  ros::spin ();
}
