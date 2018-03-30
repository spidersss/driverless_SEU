#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  
#define pi 3.1415926

ros::Publisher pub;
ros::Publisher pubxyz;
ros::Publisher pubtheta;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	std_msgs::Float64MultiArray theta_array;
	sensor_msgs::PointCloud2 output;
	sensor_msgs::PointCloud2 outputxyz;
	PointCloud::Ptr cloudxyz(new PointCloud);
	PointCloud::Ptr cloud_center(new PointCloud);// 存储每个类的质心
	pcl::PointXYZ point_center;//存储质心
	
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	PointCloud::Ptr cloud_filtered(new PointCloud);
	
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	pcl::fromPCLPointCloud2(*cloud, *cloudxyz);
  // 创建用于提取搜索方法的kdtree树对象
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloudxyz);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
  ec.setClusterTolerance (0.2);                     // 设置近邻搜索的搜索半径为0.2m
  ec.setMinClusterSize (5);                 //设置一个聚类需要的最少的点数目为100
  ec.setMaxClusterSize (250);               //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod (tree);                    //设置点云的搜索机制
  ec.setInputCloud (cloudxyz);
  ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
  //迭代访问点云索引cluster_indices,直到分割处所有聚类
  int j = 0;
  int count = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  { 
  	//迭代容器中的点云的索引，并且分开每个点云的高度值
  	count = 0;
  	point_center.x = 0;
  	point_center.y = 0;
  	point_center.z = 0;
  	std::cout<<"id:"<<j+1<<"\t";
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)	{
     //设置保存点云的属性问题
	count ++;
    cloud_filtered->points.push_back (cloudxyz->points[*pit]);
    
    point_center.x += cloudxyz->points[*pit].x;
    point_center.y += cloudxyz->points[*pit].y;
    point_center.z = 0.0;
    }
    point_center.x /=  (double)count;
    point_center.y /=  (double)count;
    cloud_center->points.push_back(point_center);
    
  j++;
     
     std::cout<<"number:"<<count<<std::endl;
  }
   std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator iter;
	for(iter = cloud_filtered->points.begin(); iter != cloud_filtered->points.end(); iter++){
		//iter->z = (j+1)*2;
		iter->z = 0;
	}
	for(iter = cloud_center->points.begin(); iter != cloud_center->points.end(); iter++){
		std::cout<<"x:"<<iter->x<<"\t"<<"y:"<<iter->y<<std::endl;
	}
  for(int i = 0; i < cloud_center->points.size(); i += 2){
  		double center_x = (cloud_center->points[i].x+cloud_center->points[i+1].x)/2.0;
  		double center_y = (cloud_center->points[i].y+cloud_center->points[i+1].y)/2.0;
		std::cout<<"center"<<i/2+1<<"("<<center_x<<","<<center_y<<")"<<std::endl;
		double theta = (atan2(-1.0, 0) - atan2(center_y, center_x))/pi*180.0;
		std::cout<<"theta"<<i/2+1<<":"<<theta<<std::endl;
		theta_array.data.push_back(theta);
		
	}
  
  //cloud_filtered->header = cloudxyz->header;
  cloud_filtered->header.frame_id = "pandar";
  cloud_filtered->width = cloud_filtered->points.size ();
  //std::cout<<"size:"<<cloud_filtered->points.size()<<std::endl;
  cloud_filtered->height = 1;
  cloud_filtered->is_dense = true;
  pcl::toROSMsg(*cloud_filtered, output);
  pub.publish(output);
  
  //cloud_center->header = cloudxyz->header;
  cloud_center->header.frame_id = "pandar";
  cloud_center->width = cloud_center->points.size();
  cloud_center->height = 1;
  //cloud_center->points.resize(cloud_center->width * cloud_center->height);
  pcl::toROSMsg(*cloud_center, outputxyz);
  pubxyz.publish(outputxyz);
  
  pubtheta.publish(theta_array);
  
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "cluster_extraction");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("filter_z", 5, cloud_cb);
	pub = n.advertise<sensor_msgs::PointCloud2> ("points_cluster_end", 1);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("points_cluster", 1);
	pubtheta = n.advertise<std_msgs::Float64MultiArray>("target_theta", 1);
	ros::spin();
}
