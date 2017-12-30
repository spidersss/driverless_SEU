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

ros::Publisher pub;
ros::Publisher pubxyz;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	PointCloud::Ptr cloudxyz(new PointCloud);
	
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter(cloud_filtered);
	
	sensor_msgs::PointCloud2 output;
	sensor_msgs::PointCloud2 output2;
	pcl_conversions::fromPCL(cloud_filtered, output);
	
	pcl::fromPCLPointCloud2(*cloud, *cloudxyz);
	
	std::cout<<cloudxyz->points.size()<<std::endl;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloudxyz->points.begin(); it != cloudxyz->points.end(); it++)
	{
		std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
		if(it->x>-3 && it->x<3 && it->y>-1 && it->y<1){
			it->x = NAN;
			it->y = NAN;
			it->z = NAN;
		}
		if(it->z < -1.0){
			it->x = NAN;
			it->y = NAN;
			it->z = NAN;
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
	std::cout<<cloudxyz->points.size()<<std::endl;
	pcl::toROSMsg(*cloudxyz, output2);

	pub.publish(output);
	pubxyz.publish(output2);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pcl_test");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("rslidar_points", 1, cloud_cb);
	pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("filter_z", 1);
	ros::spin();
}
