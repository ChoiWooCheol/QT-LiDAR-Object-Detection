#ifndef __QT_DETECT_NODE__
#define __QT_DETECT_NODE__

#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <queue>


/* Quadtree header */
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<vector>
#include<string>
#include<cmath>
#include<Eigen/Dense>
#include<pcl_conversions/pcl_conversions.h>
#include<ctime>
#include<cstdlib>
#include"qt_header/qt_detect_params.hpp"

/* obb_generator header */
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <obb_generator_msgs/cloudArray.h>


using namespace std::chrono_literals;
using namespace pcl;
using namespace std;

struct XYI{
    int count;
    double x, y, z;
    double maxZ;
};

extern jsk_recognition_msgs::BoundingBoxArray totalBox;
extern jsk_recognition_msgs::BoundingBoxArray clusteredBox;
extern std::vector< std::vector<XYI> > total_pixel;
extern double boxSize;
extern double default_x;
extern double default_y;
extern double default_size;
extern int start_Xindex;
extern int start_Yindex;
extern int default_pixel;
extern double box_height;
extern double box_z;
extern double correction_eps;

class BOX{
public:
    BOX(double xpose, double ypose, double bsize, int pixel)
        : Xpose(xpose)
        , Ypose(ypose)
        , size(bsize)
        , pixelSize(pixel) 
    {}
    
    BOX() 
        : Xpose(default_x)
        , Ypose(default_y)
        , size(default_size)
        , pixelSize(default_pixel)
    {}

    ~BOX() {}

    BOX getUR();
    BOX getUL();
    BOX getDR();
    BOX getDL();

public:
    double size;
    double Xpose, Ypose;
    int pixelSize;
};

class QuadNode{
public:
    QuadNode()
    {
        this->Children[0] = NULL;
        this->Children[1] = NULL;
        this->Children[2] = NULL;
        this->Children[3] = NULL;

        rect = BOX();
        HasChildren = false;
        HasPoints = true;
    }

    QuadNode(BOX& re)
    {
        this->Children[0] = NULL;
        this->Children[1] = NULL;
        this->Children[2] = NULL;
        this->Children[3] = NULL;

        rect = re;
        HasChildren = false;
        HasPoints = true;
    }

    ~QuadNode(){}

    int divideQuadTree();
    bool check_child(std::vector< std::vector<XYI> >& in_pixel, BOX& in_box);
    inline bool check_threshold(BOX& in_box, int in_Cnt);

public:
    double boxPose_x, boxPose_y;
    bool HasChildren, HasPoints;
    BOX rect;
    QuadNode* Children[4];
    
};

class BoxCluster{
public:
    BoxCluster() : filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>){
        obb_pub = nh.advertise<obb_generator_msgs::cloudArray>("/obb_cluster", 1);
        clusteredBox.boxes.resize(0);
        clusteredBox.header.seq = totalBox.header.seq;
        clusteredBox.header.stamp = ros::Time();
        clusteredBox.header.frame_id = "velodyne";
        cloud_arr_msg.cloudArray.clear();
        cloud_arr_msg.cloudArray.resize(0);
        // grouping_count = 0;
    }

    ~BoxCluster(){

    }

    void set_msg(sensor_msgs::PointCloud2& in_msg);
    void grouping(int index, 
                  int box_label, 
                  pcl::PointCloud<pcl::PointXYZI>& in_points, 
                  double& longest_local_z);
    void makeGroup();
    bool check_dist(jsk_recognition_msgs::BoundingBox& in_box1, 
                    jsk_recognition_msgs::BoundingBox& in_box2);

private:
    ros::Publisher obb_pub;
    ros::NodeHandle nh;

    obb_generator_msgs::cloudArray cloud_arr_msg;
    sensor_msgs::PointCloud2 points_msg;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;
};

class QuadTree{
public:
    QuadTree() 
        : seq(0)
        , private_nh("~")
    {
        CheckParams(private_nh);
        detect_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(pubtopic.c_str(), 1);
        init_pixel();
    }

    void CheckParams(ros::NodeHandle& _nh);
    void pointCallBack(sensor_msgs::PointCloud2* in_points, 
                        sensor_msgs::PointCloud2* pcl_points);
    void init_pixel();

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    
    ros::Subscriber filtered_sub;
    ros::Publisher detect_pub;


    std::string pubtopic, subtopic;
    int point_pixel_x, point_pixel_y;
    int pixel_Xmax;
    int pixel_Ymax;
    uint seq;
};


#endif