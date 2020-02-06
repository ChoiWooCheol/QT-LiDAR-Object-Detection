#include "qt_header/qt_detect_node.hpp"
#include "qt_header/qt_detect_params.hpp"


extern jsk_recognition_msgs::BoundingBoxArray totalBox;
extern std::vector< std::vector<XYI> > total_pixel;
extern double boxSize;
extern int start_Xindex;
extern int start_Yindex;


int QuadNode::divideQuadTree()
{
    BOX box = rect;
    BOX URbox = box.getUR();
    BOX ULbox = box.getUL();
    BOX DRbox = box.getDR();
    BOX DLbox = box.getDL();

    if(URbox.pixelSize < 1) return 0;

    HasChildren = true;
    HasPoints = false;

    Children[0] = new QuadNode(URbox);
    Children[1] = new QuadNode(ULbox);
    Children[2] = new QuadNode(DRbox);
    Children[3] = new QuadNode(DLbox);

    if(check_child(total_pixel, URbox)) Children[0]->divideQuadTree(); 
    delete Children[0];

    if(check_child(total_pixel, ULbox)) Children[1]->divideQuadTree(); 
    delete Children[1];

    if(check_child(total_pixel, DRbox)) Children[2]->divideQuadTree(); 
    delete Children[2];

    if(check_child(total_pixel, DLbox)) Children[3]->divideQuadTree(); 
    delete Children[3];

    return 1;
}

bool QuadNode::check_child(std::vector< std::vector<XYI> >& in_pixel, BOX& in_box)
{
    int pixelCnt = 0;
    int start_X, start_Y;
    double longest_z = -1.785;
    double result_Z = 0, total_Z =0;

    start_Y = start_Yindex - (in_box.Ypose / boxSize);
    start_X = start_Xindex + (in_box.Xpose / boxSize);
    start_Y = start_Y - (in_box.pixelSize / 2) + 1;
    start_X = start_X - (in_box.pixelSize / 2);

    for(uint i = start_Y; i < start_Y + in_box.pixelSize; ++i){
        for(uint j = start_X; j < start_X + in_box.pixelSize; ++j){
            if(in_pixel[i][j].maxZ > longest_z) longest_z = in_pixel[i][j].maxZ;
            pixelCnt += in_pixel[i][j].count;
            total_Z += in_pixel[i][j].z;
        }
    }

    result_Z = total_Z / pixelCnt + (longest_z + 1.785) / 2;
    /* if point exists in box which is minimum size box, bounding box is maked */
    if (pixelCnt == 0) return false;
    else if(check_threshold(in_box, pixelCnt)) return true;
    else if(!check_threshold(in_box, pixelCnt)){
        jsk_recognition_msgs::BoundingBox box_s;
        box_s.header.frame_id = "velodyne";
        box_s.header.seq = 0;
        box_s.header.stamp = ros::Time();
        box_s.dimensions.x = in_box.size;
        box_s.dimensions.y = in_box.size;
        box_s.dimensions.z = longest_z + 1.785;//Try
        //box_s.dimensions.z = 0.0001;

        box_s.pose.position.x = in_box.Xpose;
        box_s.pose.position.y = in_box.Ypose;
        box_s.pose.position.z = result_Z;// + (longest_z + 1.785) / 2;//Try
        //box_s.pose.position.z = -1.785;// + (longest_z + 1.785) / 2;//Try

        box_s.pose.orientation.x = 0.0;
        box_s.pose.orientation.y = 0.0;
        box_s.pose.orientation.z = 0.0;
        box_s.pose.orientation.w = 0.0;
        box_s.value = 1;
        box_s.label = 1;
        totalBox.boxes.emplace_back(box_s);
        return false;
    }
    else{
        ROS_ERROR("check children error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
        exit(1);
    }
}

inline bool QuadNode::check_threshold(BOX& in_box, int in_Cnt)
{
    if(LEVEL0_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL0_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL1_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL1_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL2_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL2_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL3_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL3_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL4_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL4_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL5_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL5_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL6_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL6_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else if(LEVEL7_BOX_PIXEL == in_box.pixelSize){
        if(LEVEL7_THRESHOLD > in_Cnt) return true;
        else return false;
    }
    else{
        ROS_ERROR("check_threshold error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
        exit(1);
    }
}