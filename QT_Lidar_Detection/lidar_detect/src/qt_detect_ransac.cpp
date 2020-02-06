#include "qt_header/qt_detect_ransac.hpp"

void Plane::callback(const sensor_msgs::PointCloud2::ConstPtr &ptr)
{
    sensor_msgs::PointCloud2 point_msg, filtered_msg, pcl_msg;

    pcl::fromROSMsg(*ptr, scan);
    pcl::fromROSMsg(*ptr, filterd_scan);
    pcl::fromROSMsg(*ptr, pcl_scan);

    filterd_scan.clear();
    pcl_scan.clear();

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
    vg.setLeafSize(0.10f,0.10f,0.10f);//set the voxel grid size //10cm
    vg.filter(*cloud_filtered);//create the filtering object

    make_plane_RANSAC();
    projection_onto_plane();

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filterd_scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
    pcl::toROSMsg(*scan_ptr, filtered_msg);
    pcl::toROSMsg(*pcl_ptr, pcl_msg);

    QT->pointCallBack(&filtered_msg, &pcl_msg);
}

void Plane::make_plane_RANSAC()
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    pcl::PointCloud<pcl::PointXYZ> filtered_points_cloud_z;
    
    // z 값이 filtering 된 point들을 가지고 pointcloud 만드는 작업. RANSAC 알고리즘에 넣어주기 위해
    for(int k = 0; k < cloud_filtered->points.size(); ++k)
    {
        if(fabs(cloud_filtered->points[k].x) < 10 && fabs(cloud_filtered->points[k].y) < 10 && cloud_filtered->points[k].z < -1.5)
        {
            pcl::PointXYZ z_filtered_point;
            z_filtered_point.x = cloud_filtered->points[k].x;
            z_filtered_point.y = cloud_filtered->points[k].y;
            z_filtered_point.z = cloud_filtered->points[k].z;
            filtered_points_cloud_z.push_back(z_filtered_point);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points_cloud_z));
    seg.setInputCloud (point_ptr);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    normal_vector.x = coefficients->values[0];
    normal_vector.y = coefficients->values[1];
    normal_vector.z = coefficients->values[2];
    D = (-1)*coefficients->values[3];
    normal_vector_queue.push(normal_vector);
    D_queue.push(D);
    extract_normal_vector(); // normal_vector의 n개의 평균을 구한다.
}

void Plane::extract_normal_vector()
{
    if(normal_vector_queue.size() == normal_vector_queue_size && D_queue.size() == normal_vector_queue_size)
    {
        float sum_x = 0.0;
        float sum_y = 0.0;
        float sum_z = 0.0;
        float sum_d = 0.0;

        for(int k = 0; k<normal_vector_queue.size(); ++k)
        {
            sum_x += normal_vector_queue.front().x;
            sum_y += normal_vector_queue.front().y;
            sum_z += normal_vector_queue.front().z;
            sum_d += D_queue.front();
        }
        //다시 갱신.
        normal_vector.x = sum_x / normal_vector_queue.size();
        normal_vector.y = sum_y / normal_vector_queue.size();
        normal_vector.z = sum_z / normal_vector_queue.size();
        D = sum_d /D_queue.size();
        
        //최신의 4개의 data point를 가지고 평균을 내기때문
        normal_vector_queue.pop(); //맨 앞 원소 제거
        D_queue.pop();
    }
}

void Plane::projection_onto_plane()
{
    Eigen::Vector4f coeffs;
    coeffs << normal_vector.x, normal_vector.y, normal_vector.z, -D;

    for(size_t i = 0; i < cloud_filtered->points.size(); ++i)
    {
        // projection이 수행되어야 하는 영역안의 points 추출 후, projection
        if(fabs(cloud_filtered->points[i].x) < x_limit && 
            fabs(cloud_filtered->points[i].y) < y_limit && 
            cloud_filtered->points[i].z < z_high_limit &&  
            cloud_filtered->points[i].z > z_low_limit )
        {
            pcl::PointXYZI projection, point;
            projection.x = cloud_filtered->points[i].x;  
            projection.y = cloud_filtered->points[i].y;
            projection.z = (-1) * (normal_vector.x * cloud_filtered->points[i].x + normal_vector.y * cloud_filtered->points[i].y - D) / normal_vector.z;
            
            //======//
            if(cloud_filtered->points[i].z > -1.5){
                point.x = cloud_filtered->points[i].x;  
                point.y = cloud_filtered->points[i].y;
                point.z = cloud_filtered->points[i].z;
            }

            //======//
            filterd_scan.points.emplace_back(point);
            projection.intensity = 2.0;
            pcl_scan.points.emplace_back(projection);
        }
    }

    filterd_scan.width = static_cast<uint32_t>(filterd_scan.points.size());
    filterd_scan.height = 1;
    sensor_msgs::PointCloud2 projected_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
    pcl::toROSMsg(*n_ptr, projected_cloud);
    projected_cloud.header.frame_id = "velodyne";
    pub4.publish(projected_cloud);
}