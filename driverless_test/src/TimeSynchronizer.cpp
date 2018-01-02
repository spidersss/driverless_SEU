#include <ros/ros.h>
#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>  
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/PointCloud2.h>    

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher imu_pub;
ros::Publisher points2_pub;  
void callback(const ImuConstPtr& imu, const PointCloud2ConstPtr& points2)  //回调中包含多个消息  
{  
	imu_pub.publish(*imu);
	points2_pub.publish(*points2);
  // Solve all of perception here...  
}  
  
int main(int argc, char** argv)  
{  
  ros::init(argc, argv, "vision_node");  
  
  ros::NodeHandle nh;  
  
  imu_pub = nh.advertise<Imu>("imu", 10);
  points2_pub = nh.advertise<PointCloud2>("points2", 10);
  
  message_filters::Subscriber<Imu> imu_sub(nh, "an_device/Imu", 1);             // topic1 输入  
  message_filters::Subscriber<PointCloud2> points2_sub(nh, "rslidar_points", 1);   // topic2 输入  
  TimeSynchronizer<Imu, PointCloud2> sync(imu_sub, points2_sub, 10);       // 同步  
  sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调  
  
  ros::spin();  
  
  return 0;  
}  
