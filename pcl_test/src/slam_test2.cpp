#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

class cloudHandler
{
public:
	cloudHandler()
	{
		lidar_sub = nh.subscribe("rslidar", 10, &cloudHandler::cloudCB, this);
		imu_sub = nh.subscribe("an_device/Imu", 10, &cloudHandler::cloudImu, this);
		//lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points2", 1);
		imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
	}
	
	void cloudCB(const sensor_msgs::PointCloud2 &input)
	{
		header = input.header;
	}
	
	void cloudImu(const sensor_msgs::Imu &imu)
	{
		sensor_msgs::Imu output;
		output = imu;
		output.header.stamp = header.stamp;
		imu_pub.publish(output);
	}
	
protected:
	ros::NodeHandle nh;
	ros::Subscriber lidar_sub, imu_sub;
	//ros::Publisher lidar_pub;
	ros::Publisher imu_pub;
	std_msgs::Header header;
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slam_test");
	cloudHandler handler;
	ros::spin();
	return 0;
}
