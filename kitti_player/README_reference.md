# kitti_player

From version 2, this node aims to play the whole kitti data into ROS (Color/Grayscale images, Velodyne scan as PCL, sensor_msgs/Imu Message, GPS as sensor_msgs/NavSatFix Message). 

 * http://www.ira.disco.unimib.it/kitti_player
 * https://github.com/iralabdisco/kitti_player

=========

Kitti_player, a player for KITTI raw datasets
Datasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php

```
Allowed options:
help           h    help message
directory      d    *required* - path to the kitti dataset Directory
frequency      f    set replay Frequency
all            a    replay All data
velodyne       v    replay Velodyne data
gps            g    replay Gps data
imu            i    replay Imu data
grayscale      G    replay Stereo Grayscale images
color          C    replay Stereo Color images
viewer         V    enable image viewer
timestamps     T    use KITTI timestamps
stereoDisp     s    use pre-calculated disparities
viewDisp       D    view loaded disparity images
frame          F    start playing at frame...
gpsPoints      p    publish GPS/RTK markers to RVIZ, having reference frame as <reference_frame> [example: -p map]
synchMode      S    Enable Synch mode (wait for signal to load next frame [std_msgs/Bool "data: true"]

kitti_player needs a directory tree like the following:
└── 2011_09_26_drive_0001_sync
    ├── image_00              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_01              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_02              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_03              
    │   └── data              
    │   └ timestamps.txt      
    ├── oxts                  
    │   └── data              
    │   └ timestamps.txt      
    ├── velodyne_points       
    │   └── data              
    │     └ timestamps.txt    
    └── calib_cam_to_cam.txt  
```
