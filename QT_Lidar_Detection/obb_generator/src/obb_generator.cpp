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
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

using namespace std::chrono_literals;

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cmath>
#include <chrono>

/* cloudclusterArray */
#include <autoware_msgs/CloudClusterArray.h>
#include <op_ros_helpers/op_ROSHelpers.h>
#include <rtp/rtp_helper.h>


#define USE_VECTORMAP 0
#define USE_AUTOWARE 1
#define MODE 0 // 1 is AABB mode
               // 0 is OBB mode
#define MAX_CLUSTER_SIZE 100

// #define my_car_width 1.5 // 가로
// #define my_car_height 4.0 // 세로

#define my_car_width 3.0 // 가로
#define my_car_height 6.0 // 세로
//for labeling
#define detection_x_len 10
#define detection_y_len 100

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

        rh_ptr_.reset(new rtp::RTPHelper(nh));
        
        obb_sub = nh.subscribe(sub_name.c_str(), 100, &ObbGenerator::obb_callback, this);
        obbArr_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(pub_name.c_str(), 100);
        to_lidar_kf_contour_track_pub = nh.advertise<rtp::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 10); // rtp
        autoware_detect_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 100);
        
        transform_obbArr_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/transform_obb_boxes", 10); //test

        rh_ptr_->spin();
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

        // obb_boxes
        pcl::PointCloud<pcl::PointXYZ> scan;
        jsk_recognition_msgs::BoundingBox obb;
        jsk_recognition_msgs::BoundingBoxArray obb_arr;

        obb_arr.header = in_obb->header;
        int z_idx = 0;
        
        for (auto& cloudarr : in_obb->cloudArray){
    
            pcl::fromROSMsg(cloudarr, scan);
            if(scan.points.size() > MAX_CLUSTER_SIZE) continue;

            std::vector<cv::Point2f> points;
            for(unsigned int i = 0; i < scan.points.size(); i++)
            {
                cv::Point2f pt;
                pt.x = scan.points[i].x;
                pt.y = scan.points[i].y;
                points.push_back(pt);
            }

            std::vector<cv::Point2f> hull;
            cv::convexHull(points, hull);

            cv::RotatedRect box = minAreaRect(hull);
            
            if(fabs(box.center.x) < my_car_height && fabs(box.center.y) < my_car_width) 
            {
                z_idx++;
                continue;
            }
            if(in_obb->zArray[z_idx] <= 0.001) 
            {
                z_idx++;
                continue;
            }
            double rz = box.angle *3.14 / 180;
            obb.header = in_obb->header;
            obb.pose.position.x = box.center.x;
            obb.pose.position.y = box.center.y;
            obb.pose.position.z = (in_obb->zArray[z_idx] / 2) - 1.785;
            obb.dimensions.x = box.size.width;
            obb.dimensions.y = box.size.height;
            obb.dimensions.z = in_obb->zArray[z_idx];

            // set bounding box direction
            tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
            tf::quaternionTFToMsg(quat, obb.pose.orientation);

            ++z_idx;
            obb_arr.boxes.emplace_back(obb);
        }


        // cloud_clusters
        
        autoware_msgs::CloudClusterArray cloudclusterarray;
        autoware_msgs::CloudCluster cloudcluster;
        
        jsk_recognition_msgs::BoundingBox obb2; //test
        jsk_recognition_msgs::BoundingBoxArray obb_arr2; //test
        obb_arr2.header = in_obb->header;
        obb_arr2.header.frame_id = "/map";

        int id_ = 0;
        z_idx = 0;
        cloudclusterarray.header = in_obb->header;
        cloudclusterarray.header.frame_id = "/map";
        
        srand((unsigned)time(NULL));

        for (auto& cloudarr : in_obb->cloudArray){

            pcl::fromROSMsg(cloudarr, scan);
            if(scan.points.size() > MAX_CLUSTER_SIZE) continue;

            pcl::PointCloud<pcl::PointXYZ> tmp;
            pcl::PointCloud<pcl::PointXYZI> transformed_scan;
            tf::StampedTransform transform;
	        PlannerHNS::ROSHelpers::GetTransformFromTF("/map", "/velodyne", transform);
            pcl_ros::transformPointCloud(scan, tmp, transform);

            pcl::copyPointCloud(tmp, transformed_scan);

            double x_ = transform.getOrigin().getX();
            double y_ = transform.getOrigin().getY();

            std::vector<cv::Point2f> points;
            double z_pose;

            for(unsigned int i = 0; i < transformed_scan.points.size(); i++)
            {
                cv::Point2f pt;
                pt.x = transformed_scan.points[i].x;
                pt.y = transformed_scan.points[i].y;
                z_pose = transformed_scan.points[i].z;
                points.push_back(pt);
            }

            sensor_msgs::PointCloud2 transfromed_msg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(transformed_scan));
            pcl::toROSMsg(*scan_ptr, transfromed_msg);

            std::vector<cv::Point2f> hull;
            cv::convexHull(points, hull);

            cv::RotatedRect box = minAreaRect(hull);
            
            if(box.center.x < x_ + my_car_height && box.center.x > x_ - my_car_height 
                && box.center.y <  y_ + my_car_width && box.center.y > y_ - my_car_width) 
            {
                //cout << box.center.x << " , " << box.center.y << endl;
                z_idx++;
                continue;
            }

            if(in_obb->zArray[z_idx] <= 0.001) 
            {
                z_idx++;
                continue;
            }

            // set bounding box direction
            double rz = box.angle *3.14 / 180;
            tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
            tf::quaternionTFToMsg(quat, obb.pose.orientation);
            tf::quaternionTFToMsg(quat, obb2.pose.orientation);
            cloudcluster.header = in_obb->header;
            cloudcluster.header.frame_id = "/map";
            cloudcluster.id = id_;
            cloudcluster.label = "unknown";
            
            cloudcluster.score = 0.0;
            cloudcluster.cloud = transfromed_msg;
            cloudcluster.centroid_point.point.x = box.center.x;
            cloudcluster.centroid_point.point.y = box.center.y;
            cloudcluster.centroid_point.point.z = z_pose + ((in_obb->zArray[z_idx] / 2));
            cloudcluster.estimated_angle = tf::getYaw(obb.pose.orientation);
            cloudcluster.dimensions.x = box.size.width;
            cloudcluster.dimensions.y = box.size.height;
            cloudcluster.dimensions.z = in_obb->zArray[z_idx];
            cloudcluster.indicator_state = 3; // None
            cloudcluster.avg_point.point.x = box.center.x;
            cloudcluster.avg_point.point.y = box.center.y;
            cloudcluster.avg_point.point.z = z_pose + ((in_obb->zArray[z_idx] / 2));
            ++z_idx;
            ++id_;
            cloudclusterarray.clusters.push_back(cloudcluster);

            //test
            obb2.header = in_obb->header;
            obb2.header.frame_id = "/map";
            obb2.pose.position.x = box.center.x;
            obb2.pose.position.y = box.center.y;
            obb2.pose.position.z = z_pose + ((in_obb->zArray[z_idx] / 2));
            obb2.dimensions.x = box.size.width;
            obb2.dimensions.y = box.size.height;
            obb2.dimensions.z = in_obb->zArray[z_idx];

            obb_arr2.boxes.emplace_back(obb2);
        }
        
        // to_lidar_kf_contour_track_pub.publish(cloudclusterarray);
        
        rtp::CloudClusterArray rtp_cloudclusterarray;
        rh_ptr_->messageInterface(cloudclusterarray, rtp_cloudclusterarray, rtp::RTPHelper::Sensor::ACTIVATED);
        to_lidar_kf_contour_track_pub.publish(rtp_cloudclusterarray);
        transform_obbArr_pub.publish(obb_arr2);
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
    ros::Publisher to_lidar_kf_contour_track_pub;

    ros::Publisher transform_obbArr_pub; //test

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
    rtp::RTPHelperPtr rh_ptr_;
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
    
    // ros::spin();
    return (0);
}
