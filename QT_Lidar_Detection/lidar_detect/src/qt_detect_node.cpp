#include "qt_header/qt_detect_node.hpp"
#include "qt_header/qt_detect_ransac.hpp"

extern std::vector< std::vector<XYI> > total_pixel;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "qt_detect_node");
    totalBox.boxes.resize(0);
    totalBox.header.seq = 0;
    totalBox.header.stamp = ros::Time();
    totalBox.header.frame_id = "velodyne";

    Plane p;
    ros::spin();
    return 0;
}