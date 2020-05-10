# kitti_player

## Attention

This package is big enough to 1.1GB because it contains kitti 2011_09_26_drive_0018_sync raw_data

## Reference

 * https://github.com/tomas789/kitti2bag
 * https://github.com/tomas789/kitti_player

## Changes
- add OpenCV in CMakeList.txt<br>
find_package( OpenCV REQUIRED )<br>
target_link_libraries(kitti_player ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Node information

Node [/kitti_player]
Publications: 
 * /kitti_player/GT_RTK [visualization_msgs/MarkerArray]
 * /kitti_player/color/left/camera_info [sensor_msgs/CameraInfo]
 * /kitti_player/color/left/image_rect [sensor_msgs/Image]
 * /kitti_player/color/left/image_rect/compressed [sensor_msgs/CompressedImage]
 * /kitti_player/color/left/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/color/left/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/color/left/image_rect/compressedDepth [sensor_msgs/CompressedImage]
 * /kitti_player/color/left/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/color/left/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/color/left/image_rect/theora [theora_image_transport/Packet]
 * /kitti_player/color/left/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/color/left/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/color/right/camera_info [sensor_msgs/CameraInfo]
 * /kitti_player/color/right/image_rect [sensor_msgs/Image]
 * /kitti_player/color/right/image_rect/compressed [sensor_msgs/CompressedImage]
 * /kitti_player/color/right/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/color/right/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/color/right/image_rect/compressedDepth [sensor_msgs/CompressedImage]
 * /kitti_player/color/right/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/color/right/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/color/right/image_rect/theora [theora_image_transport/Packet]
 * /kitti_player/color/right/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/color/right/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/grayscale/left/camera_info [sensor_msgs/CameraInfo]
 * /kitti_player/grayscale/left/image_rect [sensor_msgs/Image]
 * /kitti_player/grayscale/left/image_rect/compressed [sensor_msgs/CompressedImage]
 * /kitti_player/grayscale/left/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/grayscale/left/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/grayscale/left/image_rect/compressedDepth [sensor_msgs/CompressedImage]
 * /kitti_player/grayscale/left/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/grayscale/left/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/grayscale/left/image_rect/theora [theora_image_transport/Packet]
 * /kitti_player/grayscale/left/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/grayscale/left/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/grayscale/right/camera_info [sensor_msgs/CameraInfo]
 * /kitti_player/grayscale/right/image_rect [sensor_msgs/Image]
 * /kitti_player/grayscale/right/image_rect/compressed [sensor_msgs/CompressedImage]
 * /kitti_player/grayscale/right/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/grayscale/right/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/grayscale/right/image_rect/compressedDepth [sensor_msgs/CompressedImage]
 * /kitti_player/grayscale/right/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/grayscale/right/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/grayscale/right/image_rect/theora [theora_image_transport/Packet]
 * /kitti_player/grayscale/right/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /kitti_player/grayscale/right/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
 * /kitti_player/hdl64e [sensor_msgs/PointCloud2]
 * /kitti_player/oxts/gps [sensor_msgs/NavSatFix]
 * /kitti_player/oxts/gps_initial [sensor_msgs/NavSatFix]
 * /kitti_player/oxts/imu [sensor_msgs/Imu]
 * /kitti_player/preprocessed_disparity [stereo_msgs/DisparityImage]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /kitti_player/synch [unknown type]

Services: 
 * /kitti_player/color/left/image_rect/compressed/set_parameters
 * /kitti_player/color/left/image_rect/compressedDepth/set_parameters
 * /kitti_player/color/left/image_rect/theora/set_parameters
 * /kitti_player/color/right/image_rect/compressed/set_parameters
 * /kitti_player/color/right/image_rect/compressedDepth/set_parameters
 * /kitti_player/color/right/image_rect/theora/set_parameters
 * /kitti_player/get_loggers
 * /kitti_player/grayscale/left/image_rect/compressed/set_parameters
 * /kitti_player/grayscale/left/image_rect/compressedDepth/set_parameters
 * /kitti_player/grayscale/left/image_rect/theora/set_parameters
 * /kitti_player/grayscale/right/image_rect/compressed/set_parameters
 * /kitti_player/grayscale/right/image_rect/compressedDepth/set_parameters
 * /kitti_player/grayscale/right/image_rect/theora/set_parameters
 * /kitti_player/set_logger_level



