#include "ros/ros.h"
// darknet_ros_msgs
#include <depthGet/BoundingBoxes.h>
#include <depthGet/BoundingBox.h>
#include <depthGet/BboxL.h>
#include <depthGet/BboxLes.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
//opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <sstream>
#include <vector>

class DepthHandler
{
 public:
    DepthHandler():it(nh)
    {
     //image_transport::ImageTransport it(nh);
     sub1= it.subscribe("/kitti_player/color/left/image_rect", 200, &DepthHandler::PictureCB, this);
     sub2= nh.subscribe("/darknet_ros/bounding_boxes", 200, &DepthHandler::BBcb, this);
     sub3= nh.subscribe("ROIL2D", 10, &DepthHandler::ROIL2Dcb, this);

     pub1= it.advertise("depthMap", 1);
     //pub2= it.advertise("CutPicture", 1);

     }
     std::vector<depthGet::BboxL> BBL;
     cv::Mat dstImage;
     cv_bridge::CvImagePtr cv_ptr;
     std::vector<depthGet::BoundingBox> BBoxes;
    void PictureCB(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//      cv::imshow("OPENCV_WINDOW",cv_ptr->image); 
        dstImage= cv_ptr->image;
   	//画雷达框
         for(int i=0;i<BBL.size();++i)
        {
                float length= BBL[i].maxx- BBL[i].minx;
                float width=  BBL[i].maxy- BBL[i].miny;
               
		 cv::rectangle(dstImage,cv::Rect(BBL[i].minx,BBL[i].miny,length,width),cv::Scalar(255,0,0),1,1,0);
	
/*	//设置绘制文本的相关参数  
                std::string text = "navi:"+ std::to_string(int(BBL[i].navi))+" "+"x:"+std::to_string(int(BBL[i].centerx))+ " "+"y:"+ std::to_string(int(BBL[i].centery));
                int font_face = cv::FONT_HERSHEY_COMPLEX;
                double font_scale = 0.5;  //大小
                int thickness = 1;
                int baseline;
                //获取文本框的长宽  
                cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
                cv::Point origin;
                origin.x = BBL[i].minx;// - text_size.width / 2;  
                origin.y = BBL[i].miny - text_size.height / 2;
                cv::putText(dstImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
*/
	}	
	//画yolo框
         for(int i=0;i<BBoxes.size();++i)
        {
           if(BBoxes[i].probability>0.4)
           {
                //msg->bounding_boxes[1].xmin
                // ROS_INFO("I heard: [%d]",msg->bounding_boxes[1].xmin);       
        	float length= BBoxes[i].xmax- BBoxes[i].xmin;
                float width=  BBoxes[i].ymax- BBoxes[i].ymin;
                cv::rectangle(dstImage,cv::Rect(BBoxes[i].xmin,BBoxes[i].ymin,length,width),cv::Scalar(0,0,255),1,1,0);
    /*            //设置绘制文本的相关参数  
                std::string text = BBoxes[i].Class+"-"+std::to_string(int(100*BBoxes[i].probability))+"%";
                int font_face = cv::FONT_HERSHEY_COMPLEX;
                double font_scale = 0.5;  //大小
                int thickness = 1;
                int baseline;
                //获取文本框的长宽  
                cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
                cv::Point origin;
                origin.x = BBoxes[i].xmin;// - text_size.width / 2;  
                origin.y = BBoxes[i].ymin - text_size.height / 2;
                cv::putText(dstImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);*/
          }
        }

	//==================画融合框
	finalbox fusionfbox;	
	for(int i=0;i<BBoxes.size();++i)
	{
	if(BBoxes[i].probability>0.4)
           {
		float lengthc= BBoxes[i].xmax- BBoxes[i].xmin;
                float widthc=  BBoxes[i].ymax- BBoxes[i].ymin;
	  	for(int k=0;k<BBL.size();++k)
	        {
                float lengthl= BBL[k].maxx- BBL[k].minx;
                float widthl=  BBL[k].maxy- BBL[k].miny;
		
		bool fumaxx=(BBoxes[i].xmax-BBL[k].maxx<lengthc/2)&&(BBoxes[i].xmax-BBL[k].maxx>-lengthc/2);
		bool fuminx=(BBoxes[i].xmin-BBL[k].minx<lengthc/2)&&(BBoxes[i].xmin-BBL[k].minx>-lengthc/2);
 		bool fuminy=(BBoxes[i].ymin-BBL[k].miny<widthc/2)&&(BBoxes[i].ymin-BBL[k].miny>-widthc/2);
		bool fumaxy=(BBoxes[i].ymax-BBL[k].maxy<widthc/2)&&(BBoxes[i].ymax-BBL[k].maxy>-widthc/2);

		if(fumaxx&&fuminx&&fuminy&&fumaxy)
		   {
			fusionfbox.maxx=(BBoxes[i].xmax+BBL[k].maxx)/2;
			fusionfbox.maxy=(BBoxes[i].ymax+BBL[k].maxy)/2;
			fusionfbox.minx=(BBoxes[i].xmin+BBL[k].minx)/2;
			fusionfbox.miny=(BBoxes[i].ymin+BBL[k].miny)/2;
			fusionfbox.centerx=BBL[k].centerx;
			fusionfbox.centery=BBL[k].centery;
			fusionfbox.navi=BBL[k].navi;
			//fusionfbox.probability=BBoxes[i].probability;
			fusionfbox.Class=BBoxes[i].Class;	
			//=======draw
			float lengthf= fusionfbox.maxx- fusionfbox.minx;
                	float widthf=  fusionfbox.maxy- fusionfbox.miny;
			cv::rectangle(dstImage,cv::Rect(fusionfbox.minx,fusionfbox.miny,lengthf,widthf),cv::Scalar(0,0,255),3,1,0);

			std::string text = fusionfbox.Class+"-"+"navi:"+ std::to_string(int(fusionfbox.navi))+" "+"x:"+std::to_string(int(fusionfbox.centerx))+ " "+"y:"+ std::to_string(int(fusionfbox.centery));
	                int font_face = cv::FONT_HERSHEY_COMPLEX;
        	        double font_scale = 0.5;  //大小
                	int thickness = 1;
	                int baseline;
        	        //获取文本框的长宽  
                	cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
	                cv::Point origin;
        	        origin.x = fusionfbox.minx;// - text_size.width / 2;  
                	origin.y = fusionfbox.miny - text_size.height / 2;
                	cv::putText(dstImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

			
			break;
		   }
		}

	   }	
	}	
	sensor_msgs::ImagePtr pubmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dstImage).toImageMsg();

        pub1.publish(pubmsg);
        dstImage.release();
	BBoxes.clear();

    }

    void ROIL2Dcb (const depthGet::BboxLes::ConstPtr& input)
    {
	BBL=input->bboxl;
    }
    void BBcb(const depthGet::BoundingBoxes::ConstPtr& msg)
    {
	//auto 
//	int *beg=begin(msg->bounding_boxes);
//	int *end=end(msg->bounding_boxes);
	BBoxes=msg->bounding_boxes;
		
/*	for(int i=0;i<msg->bounding_boxes.size();++i)
	{
	   if(msg->bounding_boxes[i].probability>0.4)
	   {
		//msg->bounding_boxes[1].xmin
		// ROS_INFO("I heard: [%d]",msg->bounding_boxes[1].xmin);	
    	float length= msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin;
       	        float width=  msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin;
		cv::rectangle(dstImage,cv::Rect(msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymin,length,width),cv::Scalar(0,0,255),2,1,0);
		//设置绘制文本的相关参数  
		std::string text = msg->bounding_boxes[i].Class+"-"+std::to_string(int(100*msg->bounding_boxes[i].probability))+"%";  
		int font_face = cv::FONT_HERSHEY_COMPLEX;   
		double font_scale = 0.5;  //大小
		int thickness = 1;  
		int baseline;  
		//获取文本框的长宽  
		cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
		cv::Point origin;   
		origin.x = msg->bounding_boxes[i].xmin;// - text_size.width / 2;  
		origin.y = msg->bounding_boxes[i].ymin - text_size.height / 2;
		cv::putText(dstImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0); 	  
	  }	
	}
//	cv::imshow("OPENCV_WINDOW",dstImage);
    
	sensor_msgs::ImagePtr pubmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dstImage).toImageMsg();

        pub1.publish(pubmsg);
	dstImage.release();*/
    }
   
    protected:
        ros::NodeHandle nh;
        ros::Subscriber sub2,sub3;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub1;
        image_transport::Publisher pub1;
        //image_transport::Publisher pub2;
        struct Bbox{
        float minx_bb =0;
        float maxx_bb =0;
        float miny_bb =0;
        float maxy_bb =0;
        int flag_bb =0;
	};
	struct finalbox{
	float maxx=0;
	float maxy=0;
	float minx=0;
	float miny=0;
	float centerx=0;
	float centery=0;
	float navi=0;
	std::string Class;
	
	};

};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "depthGet");

  DepthHandler handler;
  // Spin
  ros::spin ();

  return 0;
}


