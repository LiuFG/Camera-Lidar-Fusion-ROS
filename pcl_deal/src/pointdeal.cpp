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
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//c++
//#include <vector>
//#include <vtkAutoInit.h>
class cloudHandler
{
public:
      cloudHandler()
	{
	  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("ROI", 1000);
	  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("ROIpoint", 1000);

  // Create a ROS subscriber for the input point cloud
 	 sub = nh.subscribe ("kitti_player/hdl64e", 1000, &cloudHandler::cloud_cb,this);

	}

/*struct MyPoint
{
  float x;
  float y;
  float z;
  float k;
  float w;
};*/
float max(float a,float b)
{
float c;
if(a>b)
c=a;
else
c=b;
return c;
}
float min(float a,float b)
{
float c;
if(a<b)
c=a;
else
c=b;
return c;
}

pcl::PointCloud<pcl::PointXYZ> last_cloud;

void cloud_cb (const sensor_msgs::PointCloud2& input)
{
  // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (input, cloud);   //关键的一句数据的转换
   

  // 提取x大于5区域，即相机区域
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
  *cloudptr=cloud;
  pcl::PassThrough<pcl::PointXYZ> passx;
  passx.setInputCloud (cloudptr);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (5.0, 80.0);
  passx.setFilterLimitsNegative (false);
  passx.filter(*cloud_filtered_x); 

  //提取z大于-0.5障碍物区域
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);  
   pcl::PassThrough<pcl::PointXYZ> passz;
  passz.setInputCloud (cloudptr);
  passz.setFilterFieldName ("z");
  passz.setFilterLimits (-1.0,20.0);//雷达安装高度1.73m
  passz.setFilterLimitsNegative (false);
  passz.filter(*cloud_filtered_z);
  
  //鸟瞰图栅格化,直接提取并存储有点栅格的左下角 
  pcl::PointCloud<pcl::PointXYZRGB> POI;
  int resolution_tenmul=5;
  float POIx,POIy, POIz;

  for(int i=0;i<cloud_filtered_z->points.size();++i )
{
  int flag=1;
  //浮点到整型再到浮点
  POIx=float(int(cloud_filtered_z->points[i].x*10)/resolution_tenmul*resolution_tenmul)/10.0;
  POIy=float(int(cloud_filtered_z->points[i].y*10)/resolution_tenmul*resolution_tenmul)/10.0;
  POIz=cloud_filtered_z->points[i].z;  
  for(int k=0;k<POI.points.size();++k)
   { //排除重复投影
      if(POI.points[k].x==POIx&&POI.points[k].y==POIy)
	{
	flag=0;
        POI.points[k].z=max(POI.points[k].z,POIz);
	}	
   }
  if(flag)
    {
      pcl::PointXYZRGB POIpoint;
      POIpoint.x=POIx;
      POIpoint.y=POIy;
      POIpoint.z=POIz;
      POIpoint.r=0;
      POIpoint.g=0;
      POIpoint.b=0;
      POI.push_back(POIpoint);
    }
}

  //欧式聚类  连同域
  float thresh_dis=0.8;//resolution_tenmul*resolution_tenmul*2/100.0+resolution_tenmul/10.0;
  int label=0;
//breath-first search
 

//第二次精简的循环 
 for(int i=0;i<POI.points.size()-1;++i)
{   
    if(POI.points[i].r==0)
   {
    label=label+1;
    if(label>255)
    {POI.points[i].g=label-255;
     POI.points[i].r=255;}
    else
    {POI.points[i].r=label;}
   }
    for(int k=i+1;k<POI.points.size();++k)
  {  
      float disx=POI.points[i].x-POI.points[k].x;
      float disy=POI.points[i].y-POI.points[k].y;	
      float dis=disx*disx+disy*disy;
	if(dis<thresh_dis)
        {
	  if(POI.points[k].r==0)
	   {POI.points[k].r=POI.points[i].r;
	   POI.points[k].g=POI.points[i].g;}
	  else
	  {
	    for(int j=0;j<POI.points.size();++j)
	      {
               if(POI.points[j].r==POI.points[k].r&&POI.points[j].g==POI.points[k].g)
		{
		POI.points[j].r=POI.points[i].r;
		POI.points[j].g=POI.points[i].g;
		}
	      }
	  }  	   
	}  
   }
} 

/* //第一次聚类方法
    for(int i=0;i<POI.points.size()-1;++i)
{   
    if(POI.points[i].r==0)
   {
    label=label+1;
    POI.points[i].r=label;
   
    for(int k=i+1;k<POI.points.size();++k)
    {  
     float disx=POI.points[i].x-POI.points[k].x;
     float disy=POI.points[i].y-POI.points[k].y;  
     float dis=disx*disx+disy*disy;
    if(dis<=thresh_dis)
    {
     if(POI.points[k].r==0)
     {
       POI.points[k].r=POI.points[i].r;
      }
     else
      {
      if(POI.points[k].r!=POI.points[i].r) 
        {
          label=label-1; 
              for(int j=0;j<POI.points.size();++j)
              {
               if(POI.points[j].r==POI.points[k].r)
                {
                POI.points[j].r=POI.points[i].r;
                }
              }

	}
      }
    }
    }
   }
   else
  {
    for(int k=i+1;k<POI.points.size();++k)
   {
     float disx=POI.points[i].x-POI.points[k].x;
     float disy=POI.points[i].y-POI.points[k].y;
     float dis=disx*disx+disy*disy;
    if(dis<=thresh_dis)
    {
     if(POI.points[k].r==0)
      {
       POI.points[k].r=POI.points[i].r;
      }
     else
      {
	if(POI.points[k].r!=POI.points[i].r)
	{
  	   if(POI.points[k].r>POI.points[i].r)
           {
	      for(int j=0;j<POI.points.size();++j)
              {
               if(POI.points[j].r==POI.points[k].r)
                {
                POI.points[j].r=POI.points[i].r;
                }
              }
	   }
	  else
	   {
              for(int j=0;j<POI.points.size();++j)
              {
               if(POI.points[j].r==POI.points[i].r)
                {
                POI.points[j].r=POI.points[k].r;
                }
              }

	   }
	}
      } 
    }
   }
  }
}  

*/

 
  pcl::PointCloud<pcl::PointXYZRGB> Rec;
  pcl::PointXYZRGB recpoint;
if(label<510)
{
  for(int i=1;i<label+1;++i)
{
  int flag=0;
  float max_x=0.0,min_x=100.0,max_y=-200.0,min_y=200.0,max_z=-10;

  for(int k=0;k<POI.points.size();++k)
  {
   if((POI.points[k].r+POI.points[k].g)==i)
    {
     flag=1;
     max_x=max(max_x,POI.points[k].x);
     min_x=min(min_x,POI.points[k].x);
     max_y=max(max_y,POI.points[k].y);
     min_y=min(min_y,POI.points[k].y);
     max_z=max(max_z,POI.points[k].z);
    }
  }
 
   float width=max_x-min_x+resolution_tenmul/10.0;
   float length=max_y-min_y+resolution_tenmul/10.0;

  //移除相邻帧静态框
/*
if(!lastRec.empty())
{  for(int k=0;k<lastRec.points.size();++k)
  {
   float lastmaxx,lastminx,lastmaxy,lastminy,lastwidth,lastlength,calmaxx;
   float calminx,calmaxy,calminy;
   float area1,area2,area3;
   lastmaxx=lastRec.points[k].x-resolution_tenmul/10.0;
   lastminx=lastRec.points[k].y;
   if(lastRec.points[k].z>=0)
   lastmaxy=lastRec.points[k].z-resolution_tenmul/10.0;
   else
   lastmaxy=lastRec.points[k].z;

   lastminy=(lastRec.points[k].b-1)*lastRec.points[k].r;
   lastwidth=lastmaxx-lastminx+resolution_tenmul/10.0;
   lastlength=lastmaxy-lastminy+resolution_tenmul/10.0;
   
   //计算重叠率
   calmaxx=(lastmaxx>max_x)?lastmaxx:max_x;
   calminx=(lastminx>min_x)?lastminx:min_x;
   calmaxy=(lastmaxy>max_y)?lastmaxy:max_y; 
   calminy=(lastminy>min_y)?lastminy:min_y;

   float overlapx=lastwidth+width-(calmaxx-calminx);
   float overlapy=lastlength+length-(calmaxy-calminy);
   //具有包含关系的不能分开   
   if(overlapx>0&&overlapy>0)
    {
	area1=overlapx*overlapy;
   	area2=lastwidth*lastlength;
   	area3=width*length;	
	float ratio = area1/(area2+area3-area1);
	if(ratio>0.9)
 	{
	 flag=0;
	 break;
	}
    }
  }
}
*/
  if(flag)
  {
   float width=max_x-min_x+resolution_tenmul/10.0;
   float length=max_y-min_y+resolution_tenmul/10.0;
   float area=width*length;
  //排除大面积聚类区域与小面积聚类区域
    if(area<50&&max_z>-0.4&&max_z<2)
    { 
    recpoint.x=max_x+resolution_tenmul/10.0;
    recpoint.y=min_x;
    if(max_y>=0)    
    { 
    recpoint.z=max_y+resolution_tenmul/10.0;
    }
    else
    {
    recpoint.z=max_y;
    }
    if(min_y<=0)
    {
    recpoint.r=-min_y+resolution_tenmul/10.0;
    recpoint.b=0;
    }
    else
    {
    recpoint.r=min_y;
    recpoint.b=2;
    }
    recpoint.g=(max_z+10)*10;
  
    Rec.push_back(recpoint);
    }
  } 
}
  //记录相邻帧
 // lastRec.clear();
 // lastRec=Rec;

  // 把提取出来的内点形成的平面模型的参数发布出去
  sensor_msgs::PointCloud2 output1,output2;
  pcl::toROSMsg(Rec,output1); 
  pcl::toROSMsg(POI,output2);
  output1.header.frame_id =std::string("odom");
  output2.header.frame_id =std::string("odom");
  pub1.publish (output1);
  pub2.publish (output2);
}
}

 protected:
  	ros::NodeHandle nh;
	ros::Publisher pub1,pub2;
	ros::Subscriber sub;
//	pcl::PointCloud<pcl::PointXYZRGB> lastRec;
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PoitsCloudDeal");
  
  cloudHandler handler;

  // Spin
  ros::spin ();
  return 0;
}

