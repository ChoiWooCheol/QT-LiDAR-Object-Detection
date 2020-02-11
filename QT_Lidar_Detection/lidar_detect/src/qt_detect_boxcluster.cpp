#include "qt_header/qt_detect_node.hpp"
#include "qt_header/qt_detect_params.hpp"


extern jsk_recognition_msgs::BoundingBoxArray totalBox;
extern jsk_recognition_msgs::BoundingBoxArray clusteredBox;
extern double correction_eps;


void BoxCluster::set_msg(sensor_msgs::PointCloud2& in_msg)
{
    points_msg = in_msg;
    cloud_arr_msg.header = in_msg.header;
}

void BoxCluster::grouping(int index, 
              int box_label, 
              pcl::PointCloud<pcl::PointXYZI>& in_points, 
              double& longest_local_z)
{
    jsk_recognition_msgs::BoundingBox tmp;
    tmp = totalBox.boxes[index];
    totalBox.boxes.erase(totalBox.boxes.begin() + index);
    tmp.label = box_label;
    clusteredBox.boxes.emplace_back(tmp);
    auto point = in_points.points[0];

    if (longest_local_z < tmp.dimensions.z)
        longest_local_z = tmp.dimensions.z;

    point.x = tmp.pose.position.x + (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y + (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    in_points.points.emplace_back(point);

    point.x = tmp.pose.position.x - (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y + (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    in_points.points.emplace_back(point);

    point.x = tmp.pose.position.x + (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y - (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    in_points.points.emplace_back(point);

    point.x = tmp.pose.position.x - (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y - (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    in_points.points.emplace_back(point);

    for(int i = 0; i < totalBox.boxes.size(); ++i){
        if(check_dist(tmp, totalBox.boxes[i])){
            grouping(i, box_label, in_points, longest_local_z);
            i = 0;
        }
    }
}

void BoxCluster::makeGroup()
{
    sensor_msgs::PointCloud2 points_group;
    pcl::PointCloud<pcl::PointXYZI> in_points;
    pcl::fromROSMsg(points_msg, in_points);
    in_points.clear();
    
    int _label_;
    double longest_local_z;
    srand((unsigned)time(NULL));

    while(totalBox.boxes.size() != 0){
        longest_local_z = 0.0;
        _label_ = rand() % 255 + 1;
        grouping(0, _label_, in_points, longest_local_z);
        in_points.width = static_cast<uint32_t>(in_points.points.size());
        in_points.height = 1;
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_ptr(new pcl::PointCloud<pcl::PointXYZI>(in_points));
        vg.setInputCloud(in_ptr);
        vg.setLeafSize(0.05f,0.05f,0.05f);
        vg.filter(*filtered_cloud);

        if(filtered_cloud->points.size() == 4 && 
           filtered_cloud->points[0].x - filtered_cloud->points[1].x < 0.4)
        {
            in_points.clear();
            continue;
        }

        pcl::toROSMsg(*filtered_cloud, points_group);

        cloud_arr_msg.cloudArray.emplace_back(points_group);
        cloud_arr_msg.zArray.emplace_back(longest_local_z);
        in_points.clear();
    }

    obb_pub.publish(cloud_arr_msg);
    cloud_arr_msg.cloudArray.clear();
    cloud_arr_msg.cloudArray.resize(0);

#if POINT_DEBUG
    /* edge points Debug */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptr(new pcl::PointCloud<pcl::PointXYZI>(in_points));
    pcl::toROSMsg(*cluster_ptr, points_group);
    foo_pub.publish(points_group);
#endif

}

bool BoxCluster::check_dist(jsk_recognition_msgs::BoundingBox& in_box1, 
                jsk_recognition_msgs::BoundingBox& in_box2)
{
    double box1_size = in_box1.dimensions.x;    
    double box2_size = in_box2.dimensions.x;
    double box1_dist, box2_dist, box1_to_box2;
    double box1_Xpose, box1_Ypose, box2_Xpose, box2_Ypose;

    box1_Xpose = in_box1.pose.position.x;
    box1_Ypose = in_box1.pose.position.y;
    box2_Xpose = in_box2.pose.position.x;
    box2_Ypose = in_box2.pose.position.y;

    box1_dist = sqrt(pow(box1_size / 2, 2) + pow(box1_size / 2, 2));
    box2_dist = sqrt(pow(box2_size / 2, 2) + pow(box2_size / 2, 2));

    box1_to_box2 = sqrt(pow(box1_Xpose - box2_Xpose, 2) + pow(box1_Ypose - box2_Ypose, 2));

    if((box1_dist + box2_dist) + correction_eps >= box1_to_box2) return true; // correction_eps : error correction
    else return false;
}
