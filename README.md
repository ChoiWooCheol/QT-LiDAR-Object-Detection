# QT-LiDAR-Object-Detection
  * QT (Quad-Tree Segmentation).
  * Upgrade version of LiDAR-obstacle-dectection Repository.
  * Realtime object recognization, using only LiDAR. 
  * Available for real-time self-driving systems.   
  * More powerful than euclidean clustering detection

# Hardware
* HYUNDAI i30
* Ouster OS1 64 channel LiDAR
* Intel Core i5-8250U, 3.4Ghz
* 16G RAM
* Geforce 1050GTX

# What is changed content
* Add include files.
* Existing method did not provide minium size bounding box, but this version is providing.
* Use corvarience of points, and calculate Quaternion and Rotation information of bounding box.
* bounding box's pose has orientation values.
* if you use vector map, can change cluster size (do not generte bounding box of static objects in vectormap)

# Input topic
* '/points_raw'

# Output topic
* '/detected_boxes'
* '/obb_cluster'
* '/obb_boxes'

# MODE
* OBB MODE : Bounding boxes are minimum size and have orientation values
* AABB MODE : Bounding boxes are not minimun size and do not have orientation. but a little bit fast 
* USE_VECTORMAP MODE : be going to add

# Run 
```sh
$ roslaunch lidar_detect qt_detect_launch.launch
``` 

# Result
1. Existing method
  * no rotation
  * no minimum size box
<img src="QT_Lidar_Detection/images/compare.png" width="100%" height="100%">

2. QT-detect output
 * has rotation
 * minimum size box for clustering obj
<img src="QT_Lidar_Detection/images/qt_detect_rviz.png" width="100%" height="100%">
<img src="QT_Lidar_Detection/images/qt_obb_rviz.png" width="100%" height="100%">
