#ifndef __QT_DETECT_RANSAC__
#define __QT_DETECT_RANSAC__

#include "qt_header/qt_detect_node.hpp"
#include "qt_header/find_road_points.hpp"
/* RANSAC code */
/*
    *estimate Road Plane
    *Using RANSAC algorithm 
*/

class Plane
{
    public:
        Plane() : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
        {   
            sub = nh.subscribe("/points_raw", 10000, &Plane::callback, this);
            pub4 = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 10000);
            QT = new QuadTree();
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr &ptr);
        void make_plane_RANSAC();
        void extract_normal_vector();
        void projection_onto_plane();
        
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub4;
        ros::Subscriber sub;
        QuadTree* QT;
        geometry_msgs::Point normal_vector; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
        pcl::PointCloud<pcl::PointXYZ> scan; 
        pcl::PointCloud<pcl::PointXYZI> filterd_scan, pcl_scan;

        queue< geometry_msgs::Point > normal_vector_queue; 
        queue< float > D_queue;

        float D; 
};

#endif