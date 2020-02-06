#include "qt_header/qt_detect_node.hpp"
#include "qt_header/qt_detect_params.hpp"


jsk_recognition_msgs::BoundingBoxArray totalBox;
jsk_recognition_msgs::BoundingBoxArray clusteredBox;
std::vector< std::vector<XYI> > total_pixel;
double boxSize;
double default_x;
double default_y;
double default_size;
int start_Xindex;
int start_Yindex;
int default_pixel;
double box_height;
double box_z;
double correction_eps;


void QuadTree::CheckParams(ros::NodeHandle& _nh)
{
    if(!_nh.getParam("/quadtree_params/box_z",            box_z))          throw std::runtime_error("set box_z");
    if(!_nh.getParam("/quadtree_params/box_height",       box_height))     throw std::runtime_error("set box_height");
    if(!_nh.getParam("/quadtree_params/minimum_pixel",    boxSize))        throw std::runtime_error("set boxSize");
    if(!_nh.getParam("/quadtree_params/pixel_x",          point_pixel_x))  throw std::runtime_error("set point_pixel_x");
    if(!_nh.getParam("/quadtree_params/pixel_y",          point_pixel_y))  throw std::runtime_error("set point_pixel_y");
    if(!_nh.getParam("/quadtree_params/QTsub_topic",      subtopic))       throw std::runtime_error("set subtopic");
    if(!_nh.getParam("/quadtree_params/QTpub_topic",      pubtopic))       throw std::runtime_error("set pubtopic");
    if(!_nh.getParam("/quadtree_params/pixel_Xmax",       pixel_Xmax))     throw std::runtime_error("set pixel_Xmax");
    if(!_nh.getParam("/quadtree_params/pixel_Ymax",       pixel_Ymax))     throw std::runtime_error("set pixel_Ymax");
    if(!_nh.getParam("/quadnode_params/start_Xindex",     start_Xindex))   throw std::runtime_error("set start_Xindex");
    if(!_nh.getParam("/quadnode_params/start_Yindex",     start_Yindex))   throw std::runtime_error("set start_Yindex");
    if(!_nh.getParam("/box_params/default_x",             default_x))      throw std::runtime_error("set default_x");
    if(!_nh.getParam("/box_params/default_y",             default_y))      throw std::runtime_error("set default_y");
    if(!_nh.getParam("/box_params/default_size",          default_size))   throw std::runtime_error("set default_size");
    if(!_nh.getParam("/box_params/default_pixel",         default_pixel))  throw std::runtime_error("set default_pixel");
    if(!_nh.getParam("/boxcluster_params/correction_eps", correction_eps)) throw std::runtime_error("set correction_eps");   

    default_x      = const_cast<const double&>(default_x);
    default_y      = const_cast<const double&>(default_y);
    default_size   = const_cast<const double&>(default_size);
    box_z          = const_cast<const double&>(box_z);
    box_height     = const_cast<const double&>(box_height);
    boxSize        = const_cast<const double&>(boxSize);
    correction_eps = const_cast<const double&>(correction_eps); 
    start_Xindex   = const_cast<const int&>(start_Xindex);
    start_Yindex   = const_cast<const int&>(start_Yindex);
    point_pixel_x  = const_cast<const int&>(point_pixel_x);
    point_pixel_y  = const_cast<const int&>(point_pixel_y);
    pixel_Xmax     = const_cast<const int&>(pixel_Xmax);
    pixel_Ymax     = const_cast<const int&>(pixel_Ymax);
    default_pixel  = const_cast<const int&>(default_pixel);
}

void QuadTree::pointCallBack(sensor_msgs::PointCloud2* in_points, 
                    sensor_msgs::PointCloud2* pcl_points)
{
    QuadNode*   QN = new QuadNode();
    BoxCluster* BC = new BoxCluster();
    BC->set_msg(*in_points);
    ++seq;
    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::PointCloud<pcl::PointXYZI> pcl;
    pcl::fromROSMsg(*in_points, scan);
    pcl::fromROSMsg(*pcl_points,pcl);

    double poseX = point_pixel_x / 2;
    double poseY = (point_pixel_y / 2) - 1;
    int indexX, indexY;
    for(auto point : scan.points){
        if(fabs(point.x) < 0.1 && fabs(point.y) < 0.1) continue;
        indexX = poseX + (ceil(point.x / boxSize) - 1);
        indexY = poseY - (ceil(point.y / boxSize) - 1);
        if(total_pixel[indexY][indexX].maxZ < point.z)
            total_pixel[indexY][indexX].maxZ = point.z;
        if(total_pixel[indexY][indexX].count != 0) continue;
        total_pixel[indexY][indexX].count++;
    }
        
    for(auto point : pcl.points){
        indexX = poseX + (ceil(point.x / boxSize) - 1);
        indexY = poseY - (ceil(point.y / boxSize) - 1);
        total_pixel[indexY][indexX].z = point.z;
    }

    double maxX = pixel_Xmax; // constant -> parameter
    double maxY = pixel_Ymax; // constant -> parameter
    for(uint i = 0; i < point_pixel_y; ++i){
        maxY = maxY - boxSize / 2;
        maxX = pixel_Xmax;
        for(uint j = 0; j < point_pixel_x; ++j){
            maxX = maxX + boxSize / 2;
            if(total_pixel[i][j].count != 0){
                total_pixel[i][j].x = maxX;
                total_pixel[i][j].y = maxY;
            }
            maxX = maxX + boxSize / 2;
        }
        maxY = maxY - boxSize / 2;
    }

    QN->divideQuadTree();
    BC->makeGroup();

    detect_pub.publish(clusteredBox);

    for(uint i = 0; i < point_pixel_y; ++i)
        for(uint j = 0; j < point_pixel_x; ++j)
            if(total_pixel[i][j].count != 0){
                total_pixel[i][j].count = 0;
                total_pixel[i][j].maxZ = -1.785;
            }
        
    clusteredBox.boxes.resize(0);
    totalBox.boxes.resize(0);
    totalBox.header.seq = seq++;
    totalBox.header.stamp = ros::Time();
    totalBox.header.frame_id = "velodyne";

    delete QN;
}

void QuadTree::init_pixel()
{
    XYI xyi;
    std::vector<XYI> xyi_vec;
    double poseX = static_cast<double>(point_pixel_x);
    double poseY = static_cast<double>(point_pixel_y);
    double next_x, next_y;
    next_y = boxSize;
    for(uint i = 0; i < point_pixel_y; ++i){
        xyi_vec.resize(0);
        next_x = boxSize;
        for(uint j = 0; j < point_pixel_x; ++j){
            xyi.count = 0;
            xyi.x = -poseX + next_x;
            xyi.y = poseY - next_y;
            xyi.maxZ = -1.785;
            next_x = next_x + 2*boxSize;
            xyi_vec.emplace_back(xyi);
        }
        total_pixel.emplace_back(xyi_vec);
        next_y = next_y + 2*boxSize;
    }
}