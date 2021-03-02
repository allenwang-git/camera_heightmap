//
// Created  on 2020/12/9.
// Intel realsense435i depth data trans from ROS to opencv
//

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        //cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
        //depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);//可视化
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//获取cv::Mat类型的深度数据
//        int pic_type = depth_ptr->image.type();
//        std::cout << "the element type of depth_pic is " << pic_type << std::endl;
//        ROS_INFO("SUCCESSFULLY");
        cv::waitKey(100);//更新速度
        std::cout<<"[depth_size]: "<<depth_ptr->image.size()<<std::endl;
//        std::cout<<"[depth_data]: "<<depth_ptr->image<<std::endl;
//        int after[64][64];
//        for (int i(0);i<64;i++){
//            for (int j(0);j<64;j++){
//                after[i][j]=(after[i][j])
//            }
//        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '32fc1'.", depth_msg->encoding.c_str());
    }

}
int
main(int argc, char** argv){
    ros::init (argc, argv, "depthprocess");//声明节点的名称
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub1 = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
    ros::spin();
}