#include <iostream>
#include <ros/ros.h>
#include <cstdlib>
/* obb_generator header */
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <obb_generator_msgs/cloudArray.h>
#include <pcl_ros/point_cloud.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std::chrono_literals;

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

/* set modes */
#define USE_VECTORMAP 0
#define USE_AUTOWARE 0
#define MODE 0 // 1 is AABB mode
               // 0 is OBB mode
/* --------- */
    
#define MAX_CLUSTER_SIZE 100

class ObbGenerator{
public:
    ObbGenerator()
        : private_nh("~")
    {
        if(!private_nh.getParam("obb_cluster_topic_name", sub_name))
            throw std::runtime_error("set obb_cluster_topic_name");
        if(!private_nh.getParam("obb_boxes_topic_name", pub_name))
            throw std::runtime_error("set obb_boxes_topic_name");
        if(!private_nh.getParam("maximum_cluster_size", maximum_cluster_size))
            throw std::runtime_error("set maximum_cluster_size");
        
        obb_sub = nh.subscribe(sub_name.c_str(), 1, &ObbGenerator::obb_callback, this);
        obbArr_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(pub_name.c_str(), 1);
        autoware_detect_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
    }
    
    ~ObbGenerator(){

    }

    void publishDetectedObjects(const jsk_recognition_msgs::BoundingBoxArray& in_objects)
    {
        autoware_msgs::DetectedObjectArray detected_objects;
        detected_objects.header = in_objects.header;

        for (auto& object : in_objects.boxes)
        {
            autoware_msgs::DetectedObject detected_object;
            detected_object.header = in_objects.header;
            detected_object.label = "unknown";
            detected_object.score = 1.;
            detected_object.space_frame = in_objects.header.frame_id;
            detected_object.pose = object.pose;
            detected_object.dimensions = object.dimensions;
            // detected_object.pointcloud = in_clusters.clusters[i].cloud;
            // detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
            detected_object.valid = true;
            detected_objects.objects.push_back(detected_object);
        }
        autoware_detect_pub.publish(detected_objects);
    }

#if USE_VECTORMAP
    void obb_callback(){

    }
#endif

    void obb_callback(const obb_generator_msgs::cloudArray::ConstPtr& in_obb){
        pcl::PointCloud<pcl::PointXYZ> scan;
        jsk_recognition_msgs::BoundingBox obb;
        jsk_recognition_msgs::BoundingBoxArray obb_arr;

        obb_arr.header = in_obb->header;
        int z_idx = 0;
        for (auto& cloudarr : in_obb->cloudArray){
            pcl::fromROSMsg(cloudarr, scan);

            if(scan.points.size() > MAX_CLUSTER_SIZE) continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (scan));
            feature_extractor.setInputCloud (cloud);
            feature_extractor.compute ();

#if MODE    /* AABB MODE */
            feature_extractor.getAABB (min_point_AABB, max_point_AABB);
            obb.header = in_obb->header;
            obb.pose.position.x = min_point_AABB.x + (max_point_AABB.x - min_point_AABB.x) / 2;
            obb.pose.position.y = min_point_AABB.y + (max_point_AABB.y - min_point_AABB.y) / 2;
            obb.pose.position.z = min_point_AABB.z + (max_point_AABB.z - min_point_AABB.z) / 2;
            obb.dimensions.x = max_point_AABB.x - min_point_AABB.x;
            obb.dimensions.y = max_point_AABB.y - min_point_AABB.y;
            obb.dimensions.z = in_obb->zArray[z_idx];

#else       /* OBB MODE */
            feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
            obb.header = in_obb->header;
            Eigen::Quaternionf quat (rotational_matrix_OBB);
            obb.pose.orientation.x = quat.x();
            obb.pose.orientation.y = quat.y();
            obb.pose.orientation.z = quat.z();
            obb.pose.orientation.w = quat.w();
            obb.pose.position.x = position_OBB.x;
            obb.pose.position.y = position_OBB.y;
            obb.pose.position.z = position_OBB.z + (in_obb->zArray[z_idx] / 2) ;
            obb.dimensions.x = max_point_OBB.x - min_point_OBB.x;
            obb.dimensions.y = max_point_OBB.y - min_point_OBB.y;
            obb.dimensions.z = in_obb->zArray[z_idx];
#endif
            ++z_idx;
            obb_arr.boxes.emplace_back(obb);
        }

#if USE_AUTOWARE
        publishDetectedObjects(obb_arr);
#endif
        obbArr_pub.publish(obb_arr);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber obb_sub;

    ros::Publisher obbArr_pub;
    ros::Publisher autoware_detect_pub;
    std::string pub_name, sub_name;
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    double maximum_cluster_size;

#if MODE
    /* AABB mode */
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
#else
    /* OBB mode */
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
#endif
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "obb_generator");
    ObbGenerator obbgen;

    if(MODE)
        ROS_INFO("OBB GENERATOR MODE : AABB MODE");
    else
        ROS_INFO("OBB GENERATOR MODE : OBB MODE");
    
    ros::spin();
    return (0);
}