#include <iostream>
//ROS
#include <ros/ros.h>
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
// fusion_msgs
#include <opencv_deal/BboxLes.h>
#include <opencv_deal/BboxL.h>
#include <iostream>

#include <stdio.h>
#include <sstream>
#include <vector>
class OpencvHandler
{
 public:
    OpencvHandler():it(nh)
    {
     //image_transport::ImageTransport it(nh);
     sub1= it.subscribe("/kitti_player/color/left/image_rect", 1, &OpencvHandler::color_pic, this);
     sub2= nh.subscribe("ROI", 10, &OpencvHandler::ROIcb, this);
     pub1= it.advertise("ROIpicture", 1);
     pub2= nh.advertise<opencv_deal::BboxLes>("ROIL2D", 1);   

//     cv::namedWindow("OPENCV_WINDOW");
     }
//    ~OpencvHandler()  
//    {  
//      cv::destroyWindow("OPENCV_WINDOW");  
//    }
     cv::Mat dstImage;
     cv_bridge::CvImagePtr cv_ptr;
	
    void color_pic(const sensor_msgs::ImageConstPtr& msg)
    {
        try{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
	}
        catch (cv_bridge::Exception& e)  
        {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
         return;  
        }
//	cv::imshow("OPENCV_WINDOW",cv_ptr->image); 
	dstImage= cv_ptr->image;

    }

    float min8(float a,float b,float c,float d,float e,float f,float g,float h)
    {
	float end;
	end=(a<b)?a:b;
	end=(c<end)?c:end;
	end=(d<end)?d:end;
	end=(e<end)?e:end;
	end=(f<end)?f:end;
	end=(g<end)?g:end;
	end=(h<end)?h:end;
	return end;

    }
   
    float max8(float a,float b,float c,float d,float e,float f,float g,float h)
    {
        float end;
        end=(a>b)?a:b;
        end=(c>end)?c:end;
        end=(d>end)?d:end;
        end=(e>end)?e:end;
        end=(f>end)?f:end;
        end=(g>end)?g:end;
        end=(h>end)?h:end;
        return end;

    }

     void ROIcb (const sensor_msgs::PointCloud2& input)
{
     // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
      pcl::PointCloud<pcl::PointXYZRGB> Rec;
      pcl::fromROSMsg (input, Rec);   //关键的一句数据的转换
      cv::Mat projectionMatrix,cameraMatrix,velo2camera;
      std::string calibFilePath="/home/lfg/my_work/graduate/src/opencv_deal/src/calib.txt"; 

      readOdometryCalib(calibFilePath,projectionMatrix,cameraMatrix,velo2camera);

      pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
      
      for(int i=0;i<Rec.points.size();++i)
      {
	int flag=1;

        //移除相邻帧静态框
	if(!lastRec.empty())
	{  for(int k=0;k<lastRec.points.size();++k)
  		{
   		float lastmaxx,lastminx,lastmaxy,lastminy,lastwidth,lastlength,calmaxx;
   		float calminx,calmaxy,calminy;
   		float area1,area2,area3;
		float minx,maxx,miny,maxy,width,length;
   		lastmaxx=lastRec.points[k].x;
   		lastminx=lastRec.points[k].y;
   		lastmaxy=lastRec.points[k].z;
   		lastminy=(lastRec.points[k].b-1)*lastRec.points[k].r;
   		lastwidth=lastmaxx-lastminx;
   		lastlength=lastmaxy-lastminy;
		
		maxx=Rec.points[i].x;
		minx=Rec.points[i].y;
		maxy=Rec.points[i].z;
		miny=(Rec.points[i].b-1)*Rec.points[i].r;
		width=maxx-minx;
		length=maxy-miny;
   		//计算重叠率
   		//calmaxx=(lastmaxx>maxx)?lastmaxx:maxx;
   		//calminx=(lastminx>minx)?lastminx:minx;
   		//calmaxy=(lastmaxy>maxy)?lastmaxy:maxy;
   		//calminy=(lastminy>miny)?lastminy:miny;
  		//float overlapx=lastwidth+width-(calmaxx-calminx);
   		//float overlapy=lastlength+length-(calmaxy-calminy);

		bool ifmaxx=(lastmaxx-maxx<1)&&(lastmaxx-maxx>-1);		
		bool ifmaxy=(lastmaxy-maxy<1)&&(lastmaxy-maxy>-1);
		bool ifminx=(lastminx-minx<1)&&(lastminx-minx>-1);
		bool ifminy=(lastminy-miny<1)&&(lastminy-miny>-1);

   		if(ifmaxx&&ifmaxy&&ifminx&&ifminy)//overlapx>0&&overlapy>0)
    		{  //去掉包含的情况
        	  // bool ifcontain=(minx>lastminx)&&(maxx<lastmaxx)&&(miny>lastminy)&&(maxy>lastmaxy);
		  // bool ifcontainv=(minx<lastminx)&&(maxx<lastmaxx)&&(miny<lastminy)&&(maxy<lastmaxy);
		  // area1=overlapx*overlapy;
        	  // area2=lastwidth*lastlength;
        	  // area3=width*length;
        	  // float ratio = area1/(area2+area3-area1);
        	  // if(ratio>0.99)
        	//	{
         		flag=0;
         		break;
        	//	}
    		}
		}
  	}


	if(flag)
	{
	 pcl::PointXYZ temp;
	float min_z=-1.5;
	float maxxplus=3; //除10为增加高度
	temp.x= Rec.points[i].x;
	temp.y= Rec.points[i].z;
	temp.z= float(Rec.points[i].g+maxxplus)/10.0-10;
	veloCloudPtr->push_back(temp);

	temp.x= Rec.points[i].x;
        temp.y= Rec.points[i].r*(Rec.points[i].b-1);
        temp.z= float(Rec.points[i].g+maxxplus)/10.0-10;
	veloCloudPtr->push_back(temp);

        temp.x= Rec.points[i].y;
        temp.y= Rec.points[i].z;
        temp.z= float(Rec.points[i].g+maxxplus)/10.0-10;
        veloCloudPtr->push_back(temp);

        temp.x= Rec.points[i].y;
        temp.y= Rec.points[i].r*(Rec.points[i].b-1);
        temp.z= float(Rec.points[i].g+maxxplus)/10.0-10;
        veloCloudPtr->push_back(temp);

        temp.x= Rec.points[i].x;
        temp.y= Rec.points[i].z;
        temp.z= min_z;
        veloCloudPtr->push_back(temp);

        temp.x= Rec.points[i].x;
        temp.y= Rec.points[i].r*(Rec.points[i].b-1);
        temp.z= min_z;
        veloCloudPtr->push_back(temp);

        temp.x= Rec.points[i].y;
        temp.y= Rec.points[i].z;
        temp.z= min_z;
        veloCloudPtr->push_back(temp);

        temp.x= Rec.points[i].y;
        temp.y= Rec.points[i].r*(Rec.points[i].b-1);
        temp.z= min_z;
        veloCloudPtr->push_back(temp);	
        }
  }
        
	lastRec.clear();
        lastRec=Rec;

      project2Image(dstImage,veloCloudPtr,projectionMatrix);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dstImage).toImageMsg();

     // pub1.publish(cv_ptr->toImageMsg());
      pub1.publish(msg);
}
 
    bool project2Image(cv::Mat& rgbImg,pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr,cv::Mat& projectionMatrix)
    {
        if(!rgbImg.data||!veloCloudPtr)
        {
            printf("No valid rgbImage or valid point cloud data!");
            return false;
        }
  
	pcl::PointCloud<pcl::PointXYZRGB> Point2D;
	 pcl::PointXYZRGB Ptemp;

        for(size_t i = 0; i<veloCloudPtr->points.size();++i) //scan all point cloud
        {
            cv::Mat veloPoint3D(4,1,CV_32F,cv::Scalar(1.0));
            cv::Mat imgPoint2D(3,1,CV_32F);

            veloPoint3D.at<float>(0,0) = veloCloudPtr->points[i].x;
            veloPoint3D.at<float>(1,0) = veloCloudPtr->points[i].y;
            veloPoint3D.at<float>(2,0) = veloCloudPtr->points[i].z;

            //Important!! eliminate the point behind camera
            //otherwise, it would project redundant points
            if(veloCloudPtr->points[i].x < 0)
                continue;

            imgPoint2D = projectionMatrix*veloPoint3D;
            if( imgPoint2D.at<float>(2,0) == 0 )
            {
                fprintf(stderr,"the calculated 2D image points are wrong!\n");
                exit(0);
            }
            //scale transform
            imgPoint2D.at<float>(0,0) /= imgPoint2D.at<float>(2,0); //row number
            imgPoint2D.at<float>(1,0) /= imgPoint2D.at<float>(2,0); //col number

            int colTmp = int(imgPoint2D.at<float>(0,0)+0.5);
            int rowTmp = int(imgPoint2D.at<float>(1,0)+0.5);

           // if(colTmp<0||colTmp>rgbImg.cols||rowTmp<0||rowTmp>rgbImg.rows)
           // {
           //     continue;
           // }
	   
		Ptemp.x=colTmp;
		Ptemp.y=rowTmp;
		Ptemp.z=0;
		Ptemp.r=veloCloudPtr->points[i].x;
		if(veloCloudPtr->points[i].y>0)
		{Ptemp.g=veloCloudPtr->points[i].y;
		Ptemp.b=2;}
		else
		{Ptemp.g=-veloCloudPtr->points[i].y;
                Ptemp.b=0;
		}
		Point2D.push_back(Ptemp);
	}

	std::vector<Bbox> fusionbox;
	
        std::vector<Bbox> point2Dbox;
	for(int num=0;num<Point2D.points.size();num=num+8)
	{ 
	//画立方体
/*	cv::line(rgbImg,cv::Point(Point2D.points[num].x,Point2D.points[num].y),cv::Point(Point2D.points[num+1].x,Point2D.points[num+1].y),cv::Scalar(255,0,0),3);	
	cv::line(rgbImg,cv::Point(Point2D.points[num].x,Point2D.points[num].y),cv::Point(Point2D.points[num+2].x,Point2D.points[num+2].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num].x,Point2D.points[num].y),cv::Point(Point2D.points[num+4].x,Point2D.points[num+4].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+3].x,Point2D.points[num+3].y),cv::Point(Point2D.points[num+1].x,Point2D.points[num+1].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+3].x,Point2D.points[num+3].y),cv::Point(Point2D.points[num+2].x,Point2D.points[num+2].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+3].x,Point2D.points[num+3].y),cv::Point(Point2D.points[num+7].x,Point2D.points[num+7].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+5].x,Point2D.points[num+5].y),cv::Point(Point2D.points[num+1].x,Point2D.points[num+1].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+5].x,Point2D.points[num+5].y),cv::Point(Point2D.points[num+4].x,Point2D.points[num+4].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+5].x,Point2D.points[num+5].y),cv::Point(Point2D.points[num+7].x,Point2D.points[num+7].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+6].x,Point2D.points[num+6].y),cv::Point(Point2D.points[num+2].x,Point2D.points[num+2].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+6].x,Point2D.points[num+6].y),cv::Point(Point2D.points[num+4].x,Point2D.points[num+4].y),cv::Scalar(255,0,0),3);
	cv::line(rgbImg,cv::Point(Point2D.points[num+6].x,Point2D.points[num+6].y),cv::Point(Point2D.points[num+7].x,Point2D.points[num+7].y),cv::Scalar(255,0,0),3);
*/
	//画2D矩形
        float rec_minx=min8(Point2D.points[num].x,Point2D.points[num+1].x,Point2D.points[num+2].x,Point2D.points[num+3].x,Point2D.points[num+4].x,Point2D.points[num+5].x,Point2D.points[num+6].x,Point2D.points[num+7].x);
	float rec_miny=min8(Point2D.points[num].y,Point2D.points[num+1].y,Point2D.points[num+2].y,Point2D.points[num+3].y,Point2D.points[num+4].y,Point2D.points[num+5].y,Point2D.points[num+6].y,Point2D.points[num+7].y);
	float rec_maxx=max8(Point2D.points[num].x,Point2D.points[num+1].x,Point2D.points[num+2].x,Point2D.points[num+3].x,Point2D.points[num+4].x,Point2D.points[num+5].x,Point2D.points[num+6].x,Point2D.points[num+7].x);
	float rec_maxy=max8(Point2D.points[num].y,Point2D.points[num+1].y,Point2D.points[num+2].y,Point2D.points[num+3].y,Point2D.points[num+4].y,Point2D.points[num+5].y,Point2D.points[num+6].y,Point2D.points[num+7].y);
       float rec_centerx=(Point2D.points[num].r+Point2D.points[num+1].r+Point2D.points[num+2].r+Point2D.points[num+3].r+Point2D.points[num+4].r+Point2D.points[num+5].r+Point2D.points[num+6].r+Point2D.points[num+7].r)/8;
	float rec_centery=((Point2D.points[num].b-1)*Point2D.points[num].g+(Point2D.points[num+1].b-1)*Point2D.points[num+1].g+(Point2D.points[num+2].b-1)*Point2D.points[num+2].g+(Point2D.points[num+3].b-1)*Point2D.points[num+3].g+(Point2D.points[num+4].b-1)*Point2D.points[num+4].g+(Point2D.points[num+5].b-1)*Point2D.points[num+5].g+(Point2D.points[num+6].b-1)*Point2D.points[num+6].g+(Point2D.points[num+7].b-1)*Point2D.points[num+7].g)/8;
 
	//超范围处理
      		if(rec_maxx>rgbImg.cols)
               	rec_maxx= rgbImg.cols;
        	if(rec_maxy>rgbImg.rows)
            	rec_maxy=rgbImg.rows;
        	if(rec_minx<0)
            	rec_minx=0;
        	if(rec_miny<0)
            	rec_miny=0;
		        	
	//存到待处理容器
	
	 Bbox point2Dtemp;
	 point2Dtemp.minx_bb= rec_minx;
	 point2Dtemp.maxx_bb= rec_maxx;
	 point2Dtemp.miny_bb= rec_miny;
	 point2Dtemp.maxy_bb= rec_maxy;
	 point2Dtemp.flag_bb= 0;
	 point2Dtemp.centerx= rec_centerx;
	 point2Dtemp.centery= rec_centery;
		
	 point2Dbox.push_back(point2Dtemp);

	//构造全零fusionbox
	 Bbox fusiontemp; 
         fusionbox.push_back(fusiontemp);
       }
	//消除具有包含关系的box
	for(int num=0;num<fusionbox.size();++num)
	{
	    for(int i=num;i<point2Dbox.size();++i)
	    {	
		if(point2Dbox[i].flag_bb==0)
		{
		   if(fusionbox[num].flag_bb ==0)
		   {
		   point2Dbox[i].flag_bb=1;
		   fusionbox[num].maxx_bb=point2Dbox[i].maxx_bb;
		   fusionbox[num].minx_bb=point2Dbox[i].minx_bb;
		   fusionbox[num].maxy_bb=point2Dbox[i].maxy_bb;
		   fusionbox[num].miny_bb=point2Dbox[i].miny_bb;
		   fusionbox[num].flag_bb=1;
		   fusionbox[num].centerx=point2Dbox[i].centerx;
		   fusionbox[num].centery=point2Dbox[i].centery;
		   }
		 else 
		 { 
		bool ifmaxx=(point2Dbox[i].maxx_bb>fusionbox[num].maxx_bb);
		bool ifmaxy=(point2Dbox[i].maxy_bb>fusionbox[num].maxy_bb);
                bool ifminx=(point2Dbox[i].minx_bb<fusionbox[num].minx_bb);
                bool ifminy=(point2Dbox[i].miny_bb<fusionbox[num].miny_bb);
	   	bool equalmaxx=(point2Dbox[i].maxx_bb==fusionbox[num].maxx_bb);	
		bool equalmaxy=(point2Dbox[i].maxy_bb==fusionbox[num].maxy_bb);
		bool equalminx=(point2Dbox[i].minx_bb==fusionbox[num].minx_bb);
		bool equalminy=(point2Dbox[i].miny_bb==fusionbox[num].miny_bb);
		
		bool ifomaxx=(point2Dbox[i].maxx_bb<fusionbox[num].maxx_bb+20);
                bool ifomaxy=(point2Dbox[i].maxy_bb<fusionbox[num].maxy_bb+20);
                bool ifominx=(point2Dbox[i].minx_bb>fusionbox[num].minx_bb-20);
                bool ifominy=(point2Dbox[i].miny_bb>fusionbox[num].miny_bb-20);

                bool ifomaxx2=(point2Dbox[i].maxx_bb+20>fusionbox[num].maxx_bb);
                bool ifomaxy2=(point2Dbox[i].maxy_bb+20>fusionbox[num].maxy_bb);
                bool ifominx2=(point2Dbox[i].minx_bb-20<fusionbox[num].minx_bb);
                bool ifominy2=(point2Dbox[i].miny_bb-20<fusionbox[num].miny_bb);

		if((ifmaxx||equalmaxx)&&(ifmaxy||equalmaxy)&&(ifminx||equalminx)&&(ifminy||equalminy))
		{
		//吞并
		point2Dbox[i].flag_bb=1;
                fusionbox[num].maxx_bb=point2Dbox[i].maxx_bb;
                fusionbox[num].minx_bb=point2Dbox[i].minx_bb;
                fusionbox[num].maxy_bb=point2Dbox[i].maxy_bb;
                fusionbox[num].miny_bb=point2Dbox[i].miny_bb;
                   fusionbox[num].centerx=point2Dbox[i].centerx;
                   fusionbox[num].centery=point2Dbox[i].centery;

		//变那大了，就要循环重置
		i=num;
		}
		else if(!ifmaxx&&!ifmaxy&&!ifminx&&!ifminy)
		{
		//被包含
		point2Dbox[i].flag_bb=1;
		}
		else if(ifomaxx&&ifomaxy&&ifominx&&ifominy)
		{
		//重叠
	 	fusionbox[num].maxx_bb=(point2Dbox[i].maxx_bb>fusionbox[num].maxx_bb)?point2Dbox[i].maxx_bb:fusionbox[num].maxx_bb;
                fusionbox[num].minx_bb=(point2Dbox[i].minx_bb<fusionbox[num].minx_bb)?point2Dbox[i].minx_bb:fusionbox[num].minx_bb;
                fusionbox[num].maxy_bb=(point2Dbox[i].maxy_bb>fusionbox[num].maxy_bb)?point2Dbox[i].maxy_bb:fusionbox[num].maxy_bb;
                fusionbox[num].miny_bb=(point2Dbox[i].miny_bb<fusionbox[num].miny_bb)?point2Dbox[i].miny_bb:fusionbox[num].miny_bb;
		point2Dbox[i].flag_bb=1;
		i=num;
		}
		else if(ifomaxx2&&ifomaxy2&&ifominx2&&ifominy2)
		{
		fusionbox[num].maxx_bb=(point2Dbox[i].maxx_bb>fusionbox[num].maxx_bb)?point2Dbox[i].maxx_bb:fusionbox[num].maxx_bb;
                fusionbox[num].minx_bb=(point2Dbox[i].minx_bb<fusionbox[num].minx_bb)?point2Dbox[i].minx_bb:fusionbox[num].minx_bb;
                fusionbox[num].maxy_bb=(point2Dbox[i].maxy_bb>fusionbox[num].maxy_bb)?point2Dbox[i].maxy_bb:fusionbox[num].maxy_bb;
                fusionbox[num].miny_bb=(point2Dbox[i].miny_bb<fusionbox[num].miny_bb)?point2Dbox[i].miny_bb:fusionbox[num].miny_bb;
		point2Dbox[i].flag_bb=1;
		i=num;
		}
		else {}
	         }
		}
	    }
	}

	//制作掩膜
        cv::Mat cvMask= cv::Mat::ones(rgbImg.rows,rgbImg.cols,CV_8UC1);
	//在图像上画出最后结果box
	for(int i=0;i<fusionbox.size();++i)
	{
	if(fusionbox[i].flag_bb==1)
	{
	//限定坐标范围，防止超值    //补漏动
	if(fusionbox[i].maxx_bb>rgbImg.cols)
	fusionbox[i].maxx_bb=rgbImg.cols;
	if(fusionbox[i].minx_bb<0)
	fusionbox[i].minx_bb=0;
	if(fusionbox[i].maxy_bb>rgbImg.rows)
        fusionbox[i].maxy_bb=rgbImg.rows;
        if(fusionbox[i].miny_bb<0)
        fusionbox[i].miny_bb=0;

	if(fusionbox[i].maxy_bb<0)
        {
	fusionbox[i].maxy_bb=0;
	fusionbox[i].miny_bb=0;
	}
	if(fusionbox[i].maxx_bb<0)
        {
	fusionbox[i].maxx_bb=0;
	fusionbox[i].minx_bb=0;	
	}
	if(fusionbox[i].minx_bb>rgbImg.cols)
        {
	fusionbox[i].minx_bb=0;
	fusionbox[i].maxx_bb=0;
	}
	if(fusionbox[i].miny_bb>rgbImg.rows)
        {
	fusionbox[i].miny_bb=0;
	fusionbox[i].maxy_bb=0;
	}
	//std::vector<std::vector<cd::point>> contours;
	//std::vector<cv::point> pointtemp;
	
	//pointtemp.push_back(cv::point(fusionbox[i].maxx_bb,fusionbox[i].maxy_bb));
	//pointtemp.push_back(cv::point(fusionbox[i].maxx_bb,fusionbox[i].miny_bb));
	//pointtemp.push_back(cv::point(fusionbox[i].minx_bb,fusionbox[i].miny_bb));
	//pointtemp.push_back(cv::point(fusionbox[i].minx_bb,fusionbox[i].maxy_bb));
	

	float length= fusionbox[i].maxx_bb-fusionbox[i].minx_bb;
	float width=  fusionbox[i].maxy_bb-fusionbox[i].miny_bb;
//	cv::rectangle(rgbImg,cv::Rect(fusionbox[i].minx_bb,fusionbox[i].miny_bb,length,width),cv::Scalar(0,0,255),3,1,0);
	cv::Rect r1(fusionbox[i].minx_bb,fusionbox[i].miny_bb,length,width);	
	cv::Rect r2(0,0,rgbImg.cols,rgbImg.rows);
	cvMask(r1).setTo(0); //注释掉就不报错了	
	}
	}
	
	//msgtrans
	opencv_deal::BboxLes fusionmsg;
	opencv_deal::BboxL fusiontemp;
	for(int w=0; w<fusionbox.size(); ++w)
	{
	if(fusionbox[w].maxx_bb>rgbImg.cols)
          fusionbox[w].maxx_bb= rgbImg.cols;
        if(fusionbox[w].maxy_bb>rgbImg.rows)
          fusionbox[w].maxy_bb=rgbImg.rows;
        if(fusionbox[w].minx_bb<0)
          fusionbox[w].minx_bb=0;
        if(fusionbox[w].miny_bb<0)
          fusionbox[w].miny_bb=0;


	//排除异常框
	bool xcontain=fusionbox[w].minx_bb<rgbImg.cols&&fusionbox[w].maxx_bb>0;
	bool ycontain=fusionbox[w].miny_bb<rgbImg.rows&&fusionbox[w].maxy_bb>0;
	if(xcontain&&ycontain)
	{
	fusiontemp.minx=fusionbox[w].minx_bb;
	fusiontemp.miny=fusionbox[w].miny_bb;
	fusiontemp.maxx=fusionbox[w].maxx_bb;
	fusiontemp.maxy=fusionbox[w].maxy_bb;	
	fusiontemp.navi=0;
	fusiontemp.centerx=fusionbox[w].centerx;
	fusiontemp.centery=fusionbox[w].centery;
	fusionmsg.bboxl.push_back(fusiontemp);	
	}
	}
	pub2.publish(fusionmsg);
//	rgbImg.setTo(cv::Scalar(255,255,255),cvMask);	
	return true;

}
    // std::vector DecideOverlap(std::vector<Bbox> ddddd)
    // {

     

	
    // }
     bool readOdometryCalib(const std::string& calibFile,cv::Mat& projectionMatrix,cv::Mat& cameraMatrix,cv::Mat& velo2camera)
    {
        //project 3D point under camera0 coordinate to other camera image , to image of camera2 by default
        cameraMatrix = cv::Mat(3,4,CV_32F,cv::Scalar::all(0));
        velo2camera = cv::Mat(4,4,CV_32F,cv::Scalar::all(0)); //Tr,velodyne lidar point cloud to camera0 world coordinate
        velo2camera.at<float>(3,3) = 1.0;

        std::ifstream inputFile;
        inputFile.open(calibFile.c_str(),std::ios_base::in);
        if(!inputFile.is_open())
        {
            fprintf(stderr, "cannot open the calibration file: %s!\n", calibFile.c_str());
            return false;
        }

        while( !inputFile.eof() )
        {
            std::string line_tmp;
            std::getline(inputFile,line_tmp); //read one line
            std::istringstream inputString(line_tmp);//define a string stream
            std::string tag;
            inputString>>tag;

            //just read the color camera of P2 for this function
            if ( tag == "P2:")
            {
                inputString>> cameraMatrix.at<float>(0,0) >> cameraMatrix.at<float>(0,1) >> cameraMatrix.at<float>(0,2) >> cameraMatrix.at<float>(0,3)
                           >> cameraMatrix.at<float>(1,0) >> cameraMatrix.at<float>(1,1) >> cameraMatrix.at<float>(1,2) >> cameraMatrix.at<float>(1,3)
                           >> cameraMatrix.at<float>(2,0) >> cameraMatrix.at<float>(2,1) >> cameraMatrix.at<float>(2,2) >> cameraMatrix.at<float>(2,3);
            }

            if(tag == "Tr:")
            {
                inputString >> velo2camera.at<float>(0,0) >> velo2camera.at<float>(0,1) >> velo2camera.at<float>(0,2) >> velo2camera.at<float>(0,3)
			    >> velo2camera.at<float>(1,0) >> velo2camera.at<float>(1,1) >> velo2camera.at<float>(1,2) >> velo2camera.at<float>(1,3)
                            >> velo2camera.at<float>(2,0) >> velo2camera.at<float>(2,1) >> velo2camera.at<float>(2,2) >> velo2camera.at<float>(2,3);
            }

        }
        inputFile.close();

        //projectionMatrix = 3X4;
        projectionMatrix = cameraMatrix*velo2camera;//project 3D point in lidar to camera image coordinate

        return true;

    }



    protected:
        ros::NodeHandle nh;
	ros::Subscriber sub2;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub1;
        image_transport::Publisher pub1;
	ros::Publisher pub2;
        struct Bbox{
        float minx_bb =0;
        float maxx_bb =0;
        float miny_bb =0;
        float maxy_bb =0;
        int flag_bb =0;
	float centerx;
	float centery;
        };
	pcl::PointCloud<pcl::PointXYZRGB> lastRec;	
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "showROI");

  OpencvHandler handler;
  // Spin
  ros::spin ();

  return 0;
}

