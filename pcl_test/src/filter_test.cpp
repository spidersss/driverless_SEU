#include <ros/ros.h>

#include <iostream>
#include <vector>

#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  

ros::Publisher pubxyz;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	PointCloud::Ptr cloudxyz(new PointCloud);
	
	PointCloud::Ptr cloud_filtered(new PointCloud);
	
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	
	sensor_msgs::PointCloud2 output;
	
	pcl::fromPCLPointCloud2(*cloud, *cloudxyz);
	
	std::cout<<cloudxyz->points.size()<<std::endl;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	//double lower = 0.0;
	for(it = cloudxyz->points.begin(); it != cloudxyz->points.end(); it++)
	{	
		
		if(it->x>-5 && it->x<5 && it->y<0 && it->y>-50 && it->z > -0.1 && it->z < 1 && it->x !=NAN && it->y  != NAN && it->z != NAN){
		//if(it->x>-5 && it->x<5 && it->y<0 && it->y>-50 && it->z < 1 && it->x !=NAN && it->y  != NAN && it->z != NAN){
			cloud_filtered->points.push_back (*it);
			std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
			//if((double)it->z < lower) lower = (double)it->z;
		}
		
		/*
		if(sqrt(it->x*it->x + it->y*it->y) < 3.0) 
		{
			it->x = NAN;
			it->y = NAN;
			it->z = NAN;	
		}
		*/

	}
	//std::cout<<"lower:"<<lower<<std::endl;
	cloud_filtered->header = cloudxyz->header;
	cloud_filtered->width = cloud_filtered->points.size ();
  	cloud_filtered->height = 1;
  	cloud_filtered->is_dense = false;
	std::cout<<cloud_filtered->points.size()<<std::endl;
	pcl::toROSMsg(*cloud_filtered, output);

	pubxyz.publish(output);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pcl_test");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("pandar_points", 1, cloud_cb);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("filter_z", 1);
	ros::spin();
}
