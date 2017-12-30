#include <ros/ros.h>

#include <iostream>
#include <vector>

#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  

ros::Publisher pubxyz;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	PointCloud::Ptr cloudxyz(new PointCloud);
	
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	
	sensor_msgs::PointCloud2 output;
	
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize (1.5f, 1.5f, 1.5f);
	sor.filter(cloud_filtered);//后期将点连成线，形成赛道
	
	pcl::fromPCLPointCloud2(cloud_filtered, *cloudxyz);
	
	std::cout<<cloudxyz->points.size()<<std::endl;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloudxyz->points.begin(); it != cloudxyz->points.end(); it++)
	{	
		if(it->x != 0 && !std::isnan(it->x))
		{
			std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
		}
	}

	//pub.publish(output);
	pcl::toROSMsg(*cloudxyz, output);
	pubxyz.publish(output);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "distance");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("filter_z", 1, cloud_cb);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("distance_test", 1);
	ros::spin();
}
