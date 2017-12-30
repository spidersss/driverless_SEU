#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(input, cloud);
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloud.points.begin(); it != cloud.points.end(); it++)
	{
		std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
	}
	pcl::io::savePCDFileASCII("pcd_record.pcd", cloud);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_write");
	
	ros::NodeHandle nh;
	ros::Subscriber bat_sub = nh.subscribe("rslidar_points", 10, cloudCB);
	
	ros::spin();
	return 0;
}
