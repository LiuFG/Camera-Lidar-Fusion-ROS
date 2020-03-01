#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h> 
#include <vector>
class cloudHandler
{
public:
    cloudHandler()
    : viewer("ROI Viewer")
    {
	//kitti_player/hdl64e   ROIpoint
        pcl_sub1 = nh.subscribe("kitti_player/hdl64e", 1000, &cloudHandler::cloudCB, this);
        pcl_sub2 = nh.subscribe("ROI", 1000, &cloudHandler::RecCB, this);
	viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

//	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, outviewer);
	viewer.setBackgroundColor(0, 0, 0, outviewer);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	}
   

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
	pcl::PointCloud<pcl::PointXYZ> cloud;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr  Ptrcloud;

        pcl::fromROSMsg(input, cloud);
        //Ptrcloud=cloud.makeShared();	
        viewer.removeAllPointClouds(outviewer);
        viewer.addPointCloud<pcl::PointXYZ> (cloud.makeShared(), "cloud", outviewer);
	//viewer.updatePointCloud(cloud.makeShared(), "cloud");    
	
    }

    void RecCB(const sensor_msgs::PointCloud2 &input)
    {
	pcl::PointCloud<pcl::PointXYZRGB> Rec;
        pcl::fromROSMsg (input, Rec);   //关键的一句数据的转换
        int *beg=std::begin(shape);
	int *last=std::end(shape);
	//for(int i=0;i<shape_num;++i)
	for(int i=0;beg!=last;++i)
	{
	viewer.removeShape(std::to_string(i));
	++beg;
	}
	shape_num=0;
         for(int i=0;i<Rec.points.size();++i)
        {
           //向视窗添加立方体模型并渲染，只显示线框。若不要显示线框将下面一行代码注释即可。
            viewer.addCube(Rec.points[i].y,Rec.points[i].x,Rec.points[i].r*(Rec.points[i].b-1), Rec.points[i].z, -1.7,float(Rec.points[i].g)/10.0-10,1,1,1,std::to_string(i));
           viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,std::to_string(i));
           shape_num=shape_num+1; 
	}
 
   }

    void timerCB(const ros::TimerEvent&)
    {
	viewer.spinOnce();
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> 
  //viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

   
/*    for(int i=0;i<Rec.points.size();++i)
    {
    //向视窗添加立方体模型并渲染，只显示线框。若不要显示线框将下面一行代码注释即可。
    viewer->addCube(Rec.points[i].y,Rec.points[i].x,Rec.points[i].r, Rec.points[i].z, -1.7, Rec.points[i].g);
    
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"cube");
    }
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
   while (!viewer->wasStopped ())
   {  
     viewer->spinOnce (100);
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
*/


protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub1;
    ros::Subscriber pcl_sub2;
   // pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
    pcl::visualization::PCLVisualizer viewer; 
    int outviewer=0,shape[500];
    int shape_num=0;    
};


int main (int argc, char **argv)
{
    ros::init (argc, argv, "ROIviewer");

    cloudHandler handler;

    ros::spin();

    return 0;
}

