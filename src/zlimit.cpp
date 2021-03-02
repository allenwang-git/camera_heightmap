#include <iostream>
#include <vector>        //提供向量头文件
#include <algorithm>     // 算法头文件，提供迭代器
#include <sstream>
#include <iomanip>       //C++输出精度控制需要
//ros 相关
#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

//滤波的头文件
#include <pcl/filters/passthrough.h>
//lcm
#include "lcm/lcm-cpp.hpp"
#include "heightnew_t.hpp"

// 1012
heightnew_t heightnew_lcm;//高度地图矩阵
lcm::LCM heightmapnewLCM;
//申明发布器
ros::Publisher pub;
//声明点云矩阵
using namespace std;

//取小数位数
float round2(float dVal, short iPlaces) {
    float dRetval;
    float dMod = 0.0000001;
    if (dVal<0.0) dMod=-0.0000001;
    dRetval=dVal;
    dRetval+=(5.0/pow(10.0,iPlaces+1.0));
    dRetval*=pow(10.0,iPlaces);
    dRetval=floor(dRetval+dMod);
    dRetval/=pow(10.0,iPlaces);
    return(dRetval);
}
//点云转高度
//float pcltoHeight(int i,int j,float pcd[][3],int row){
//    vector<float> nearheight;
//    float r=0.01;//范围半径
//    float h;//该位置的近似高度
//    float ii=-0.02*i+2;//camera_frame下的x
//    float jj=-0.02*j+1;//camera_frame下的y
//    for (int k = 0; k <row; ++k) {
//        //所有点与(i,j)点的欧氏距离
////        float odis=sqrt((pcd[k][0]-ii)*(pcd[k][0]-ii)+(pcd[k][1]-jj)*(pcd[k][1]-jj));
////
//        if (abs(pcd[k][0]-ii)<=r && abs(pcd[k][1]-jj)<=r)//存入所有符合条件的点的高度信息
//        {   cout<<pcd[k][2]<<endl;
//            float hh=pcd[k][2];
//            nearheight.push_back(hh);}
//
//    }
//    cout<<"final"<<nearheight.back();
//    //对所有符合条件的高度值排序，取最大
//    sort(nearheight.begin(),nearheight.end());
//    h=nearheight.back();
//    cout<<h<<endl;
//
//    return h;
//}
//回调函数  z直通滤波+pcd预处理
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filted;//滤波后的点云
    pcl::PointCloud<pcl::PointXYZ> cloud_out;//转换格式后的点云

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // build the filter
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.0, 0.7);
    // apply filter
    pass.filter (cloud_filted);//滤波后存为cloud_filted
    pcl::fromPCLPointCloud2(cloud_filted,cloud_out);//转换格式为pcl::PointXYZ
    // save data
//    pcl::io::savePCDFileASCII ("/home/allen/catkin_ws/src/my_pcl/data/pcd_50610.pcd", cloud_out);//保存pcd
//    pcl::io::loadPCDFile ("/home/allen/catkin_ws/src/my_pcl/data/pcd_5061.pcd", cloud_out);
    // get max pcd y
    pcl::PointXYZ pcmax,pcmin;//用于存放三个轴的最大值
    pcl::getMinMax3D(cloud_out,pcmin,pcmax);//获取pc最大最小值
    double maxz=pcmax.y;

    //#########################################################//

    //  初始化pcd矩阵
    const int col=3;
    int size =cloud_out.points.size ();
    float pcd[size][col];//根据pc数据量动态分配空间

    for (int i = 0; i < size; ++i) //设置pc坐标
    {//修正坐标轴映射
        pcd[i][0]=round2(cloud_out.points[i].z ,2) ;//pcd-x ;
        pcd[i][1]=-round2(cloud_out.points[i].x ,2); //pcd-y ;
        pcd[i][2]=round2(maxz-cloud_out.points[i].y ,2); //pcd-z ;

//        cout<<pcd[i][0]<<"  "<<pcd[i][1]<<"  "<<pcd[i][2]<<endl;
//        cout<<" x "<<cloud_out.points[i].z
//            <<" y "<<cloud_out.points[i].x
//            <<" z "<<cloud_out.points[i].y<<endl;
    }

    cout<<" end loop !! & pc_size: "<<size<<endl;

    //pointcloud data to heightmapnew data
//    float heightnew[101][101];//高度地图矩阵
    float r=0.01;//近似范围 -正方形边长
    for (int m = 0; m <=100; ++m) {
        for (int n = 0; n <= 100; ++n) {
//            heightnew[m][n]=pcltoHeight(m,n,pcd,size);
            float ii=-0.02*m+2;//camera_frame下的x
            float jj=-0.02*n+1;//camera_frame下的y
            float h=0.0;//该位置的近似高度
            for (int k = 0; k <size; ++k) {
                if (abs(pcd[k][0]-ii)<=r && abs(pcd[k][1]-jj)<=r)//存入所有符合条件的点的高度信息
                {
                    if (pcd[k][2]>h)
                        h=pcd[k][2];
                }
            }
            heightnew_lcm.map[m][n]=h;
//            cout<<h<<"  ";
        }
//        cout<<endl;
    }
// lcm publish heightmap[101][101] to cheetah
    heightmapnewLCM.publish("heightmapnew", &heightnew_lcm);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_pt;
    pcl_conversions::moveFromPCL(cloud_filted, cloud_pt);

    // Publish the data
//    ros::Duration(0.1).sleep();//更新频率
    pub.publish (cloud_pt);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "zlimit");//声明节点的名称
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    // 为接受点云数据创建一个订阅节点
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("ylimited", 1, cloud_cb);
//    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("downsampled", 1, cloud_cb);
    // Create a ROS publisher for the output point cloud
    //创建ROS的发布节点
    pub = nh.advertise<sensor_msgs::PointCloud2> ("zfinal", 1);

    // 回调
    ros::spin ();
}



