#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler
{
public:
	cloudHandler()
	{
		pcl_sub = nh.subscribe("filter_z", 10, &cloudHandler::cloudCB, this);
		pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched", 1);
	}
	
	void cloudCB(const sensor_msgs::PointCloud2 &input)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> cloud_pre;//从文件中加载，利用正态分布变换进行配准（normal Distributions Transform）
		pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
		sensor_msgs::PointCloud2 output;
		
		pcl::fromROSMsg(input, cloud);
		
		cloud_pre = cloud;
		
		pcl::IterativeClosestPoint<pcl::PointXYZ,  pcl::PointXYZ> icp;
		icp.setInputSource(cloud_pre.makeShared());
		icp.setInputTarget(cloud.makeShared());
		
		icp.setMaxCorrespondenceDistance(5);
		icp.setMaximumIterations(100);
		icp.setTransformationEpsilon(1e-12);
		icp.setEuclideanFitnessEpsilon(0.1);
		
		icp.align(cloud_aligned);
		
		pcl::toROSMsg(cloud_aligned, output);
		pcl_pub.publish(output);
	}
protected:
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
	ros::Publisher pcl_pub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_matching");
	cloudHandler handler;
	ros::spin();
	return 0;
}
