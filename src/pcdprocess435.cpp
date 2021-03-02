// headers
#include <iostream>
#include <vector>        //提供向量头文件
#include <algorithm>     // 算法头文件，提供迭代器
#include <sstream>
#include <fstream>
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
#include <pcl/common/transforms.h>
//滤波的头文件
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
//lcm
#include "lcm/lcm-cpp.hpp"
#include "traversability_float_t.hpp"
#include "heightnew_t.hpp"

// 1012
heightnew_t heightnew_lcm;//高度地图矩阵
traversability_float_t trav_lcm;
lcm::LCM _lcm;
float _height_mapnew[101][101];
float _trav_map[100][100];
//申明发布器
ros::Publisher pub;
//声明点云矩阵
using namespace std;
bool b=true,b1=true,b2=true;
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

//回调函数  xyz直通滤波+pcd坐标修正+点云转高度地图+lcm发布
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);//降采样后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated(new pcl::PointCloud<pcl::PointXYZ>);//姿态变换后的点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr xpassed(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr ypassed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filted(new pcl::PointCloud<pcl::PointXYZ>);//降噪后的点云
    pcl::PointCloud<pcl::PointXYZ> cloud_out;//滤波后的点云

    // Convert ros type to PCL<xyz> data type
    pcl::fromROSMsg (*cloud_msg,*cloud);

//    downsample降采样
    // Perform the actual filtering进行一个滤波处理
    pcl::VoxelGrid<pcl::PointXYZ> sor; //创建滤波对象
    sor.setInputCloud (cloud);  //设置输入的滤波，将需要过滤的点云给滤波对象
    sor.setLeafSize (0.013f, 0.013f, 0.013f);  //设置滤波时创建的体素大小为1.2cm立方体
    sor.filter (*downsampled);//执行滤波处理，存储输出cloud_filtered

//    修正点云姿态
    float theta = -M_PI*8/45;//相机俯仰角32度
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));//生成旋转矩阵
    // 执行变换，并将结果保存在新创建的 rotated 中
    pcl::transformPointCloud (*downsampled, *rotated, transform);

//    消除噪声点--半径滤波
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器
    radiusoutlier.setInputCloud(rotated);    //设置输入点云
    radiusoutlier.setRadiusSearch(0.04);     //设置半径为100的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(4); //设置查询点的邻域点集数小于2的删除
    // 执行滤波，并将结果保存在新创建的 rotated 中
    radiusoutlier.filter(*filted);

////    x轴范围限制
//    // Perform the actual filtering
//    pcl::PassThrough<pcl::PointXYZ> xpass;//定义滤波器
//    // build the filter
//    xpass.setInputCloud (rotated);//输入要滤波的点云
//    xpass.setFilterFieldName ("z");
//    xpass.setFilterLimits (0.0, 2.0);
//    // apply filter
//    xpass.filter (*xpassed);//滤波后存为xpassed
//
////    y轴范围限制
//    // Perform the actual filtering
//    pcl::PassThrough<pcl::PointXYZ> ypass;
//    // build the filter
//    ypass.setInputCloud (xpassed);
//    ypass.setFilterFieldName ("x");
//    ypass.setFilterLimits (-1.0, 1.0);
//    // apply filter
//    ypass.filter (*ypassed);//滤波后存为ypassed
//
////    z轴范围限制
//    // Perform the actual filtering
//    pcl::PassThrough<pcl::PointXYZ> zpass;
//    // build the filter
//    zpass.setInputCloud (ypassed);
//    zpass.setFilterFieldName ("y");
//    zpass.setFilterLimits (-1.0, 0.7);
//    // apply filter
//    zpass.filter (cloud_out);//滤波后存为cloud_out

//   点云xyz范围限制
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -1.0)));  //GT表示大于等于
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 1.0)));  //LT表示小于等于
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -1.0)));  //GT表示大于等于
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.95)));  //LT表示小于等于
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));  //GT表示大于等于
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.0)));  //LT表示小于等于

    pcl::ConditionalRemoval<pcl::PointXYZ> condition;//定义滤波器
    condition.setCondition(range_condition);//设置滤波条件
    condition.setInputCloud(filted); //输入点云
//    condition.setKeepOrganized(true);
    //执行滤波并将滤波后的点云存储在xyzpassed中
    condition.filter(cloud_out);

//     save data
//    pcl::io::savePCDFileASCII ("/home/allen/catkin_ws/src/my_pcl/data/pcd_50610.pcd", cloud_out);//保存pcd
//    pcl::io::loadPCDFile ("/home/allen/catkin_ws/src/my_pcl/data/pcd_5061.pcd", cloud_out);

//   Convert from PCL<XYZ> to ROS data type
    sensor_msgs::PointCloud2 cloud_pt;
    pcl::toROSMsg (cloud_out,cloud_pt);

//     Publish the data
    pub.publish (cloud_pt);


    // get max pcd y
    pcl::PointXYZ pcmax,pcmin;//用于存放三个轴的最大值
    pcl::getMinMax3D(cloud_out,pcmin,pcmax);//获取pc最大最小值
    double maxz=pcmax.y;

    //  初始化pcd矩阵
    const int col=3;
    int size =cloud_out.points.size();
    float pcd[size][col];//根据pc数据量动态分配空间

    for (int i = 0; i < size; ++i) //设置pc坐标
    {//修正坐标轴映射
        pcd[i][0]=round2(cloud_out.points[i].z ,2) ;//pcd-x ;
        pcd[i][1]=-round2(cloud_out.points[i].x ,2); //pcd-y ;
        pcd[i][2]=round2(maxz-cloud_out.points[i].y ,2); //pcd-z ;
    }

    cout<<" point clouds num: "<<size<<endl;
if(b){
    //    pointcloud data to heightmapnew data
    float r=0.01;//近似范围 -半径
    for (int m = 0; m <=100; ++m) {
        for (int n = 0; n <= 100; ++n) {

            float ii=-0.02*m+2;//camera_frame下的x
            float jj=-0.02*n+1;//camera_frame下的y
            float h=0.001;//该位置的近似高度
            for (int k = 0; k <size; ++k) {
                if (abs(pcd[k][0]-ii)<=r && abs(pcd[k][1]-jj)<=r)//存入所有符合条件的点的高度信息
                {
                    if (pcd[k][2]>h)
                        h=pcd[k][2];
                }
            }
            if ( h<0.02) h=0.0;//1115
            heightnew_lcm.map[m][n]=h;
            _height_mapnew[m][n]=h;
            //output to file
            if (b1==true){
            std::ofstream outfile;
            outfile.open("/home/allen/heightmap.txt",ios::app);
            outfile << heightnew_lcm.map[m][n] << std::endl;
//            cout<<"ssss";
            }
        }
    }
    b1=false;


//    利用几何特征计算traversability map, 标准差，平均斜率，最大值，最小值
    float w_sd=30.,w_sl=30.,w_max=2.,w_min=2.;//score wieght
    int xnew_size=101,ynew_size=101;
    for (int i(0); i < xnew_size; ++i) {//101
        for (int j(0); j < ynew_size; ++j) {//101
//            float travmap[100][100];
            bool bb=false;
            if (i>=1&&j>=1&&i+1<xnew_size&&j+1<ynew_size)
            {   bb=true;
                float slope=0, sum=0;
                float hmax=_height_mapnew[i][j], hmin=_height_mapnew[i][j];
                float mean,sd,score;
                float tmpmap[9];
                int k=0;
                for (size_t m(i-1); m<=(i+1);m++){
                    for (size_t n(j-1); n<=(j+1);n++){
                        if (m!=i && n!=j)
//                            cout<<_height_mapnew(m,n)<<endl;
                            // slope
                            slope=abs((_height_mapnew[i][j]-_height_mapnew[m][n])/sqrt((int)((i-m)*(i-m)+(j-n)*(j-n))))+slope;
                        //max
                        if (_height_mapnew[m][n]>hmax)
                            hmax=_height_mapnew[m][n];
                        //min
                        if (_height_mapnew[m][n]<hmin)
                            hmin=_height_mapnew[m][n];
                        sum=sum+_height_mapnew[m][n];
                        tmpmap[k]=_height_mapnew[m][n];
                        k++;
                    }
                }
                //标准差
                mean=sum/9;
                sd=sqrt((tmpmap[0]-mean)*(tmpmap[0]-mean)+(tmpmap[1]-mean)*(tmpmap[1]-mean)+(tmpmap[2]-mean)*(tmpmap[2]-mean)+(tmpmap[3]-mean)*(tmpmap[3]-mean)+(tmpmap[4]-mean)*(tmpmap[4]-mean)
                        +(tmpmap[5]-mean)*(tmpmap[5]-mean)+(tmpmap[6]-mean)*(tmpmap[6]-mean)+(tmpmap[7]-mean)*(tmpmap[7]-mean)+(tmpmap[8]-mean)*(tmpmap[8]-mean));
                //斜率
                slope=slope/8;
                //traversability map
                score = w_sd*sd + w_sl*slope + w_max*(hmax-_height_mapnew[i][j]) + w_min*(_height_mapnew[i][j]-hmin);
                trav_lcm.map[i][j]=score;
                _trav_map[i][j]=score;
//                cout << "i" << i << " j" << j << " score " << trav_lcm.map[i][j] << " slope: " << slope <<" sd: "<<sd<<endl;
            }
            else if((i==0 || j==0)&&i+1<xnew_size&&j+1<ynew_size) {
                bb=true;
                trav_lcm.map[i][j] = 0.;
                _trav_map[i][j]=0.;
//                cout << "i" << i << " j" << j << " score " << trav_lcm.map[i][j] << endl;
            }

            //output to file
            if (b2== true&&bb==true)
            {
            std::ofstream outfile;
            outfile.open("/home/allen/scoremap.txt",ios::app);
            outfile << trav_lcm.map[i][j] << std::endl;

            }
        } // y loop
    } // x loop
    b2=false;
    b=false;
}
    else if (!b){
        for (int i(0); i < 101; ++i) {//101
            for (int j(0); j < 101; ++j) {//101
                heightnew_lcm.map[i][j]=_height_mapnew[i][j];
//                if (_height_mapnew[i][j]!=0)
//                    cout<<_height_mapnew[i][j];
            }
        }

        for (int i(0); i < 100; ++i) {//100
            for (int j(0); j < 100; ++j) {
                trav_lcm.map[i][j]=_trav_map[i][j];
            }
        }
    }
    // lcm publish heightmap[101][101] to cheetah
    _lcm.publish("heightmapnew", &heightnew_lcm);
// lcm publish travesabilitymap[100][100] to cheetah
    _lcm.publish("traversability_float", &trav_lcm);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcdprocess");//声明节点的名称
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    // 为接受点云数据创建一个订阅节点
//    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth/color/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    //创建ROS的发布节点
    pub = nh.advertise<sensor_msgs::PointCloud2> ("pcdfinal", 1);

    // 回调
    ros::spin();
}




