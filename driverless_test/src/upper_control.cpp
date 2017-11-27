#include "ros/ros.h"
#include "std_msgs/String.h"

#include <math.h>

#include "driverless_test/gps_data.h"
#include "driverless_test/control_data.h"
#define pi 3.1415926

driverless_test::control_data control_data;
ros::Publisher control_pub;
double endlat = 31.13123, endlon = 118.313123;
double disToend = 0.0;	
float t_yaw = 0.0;
int stop_flag = 0;
int visit_flag = 0;
float add_part = 25;
float pre_steer = 90;
float prop = 1.0;
int times = 0;
	
int speed = 10;

void controlCallback(const driverless_test::gps_data::ConstPtr& gps_msg)
{
  ROS_INFO("endlat:%f\tendlon:%f", endlat, endlon);
  if(times == 0){
	control_data.speed = speed;
	control_data.steer = 0;
	control_data.power = 1;
	times = 1;	
  }
  disToend = abs(gps_msg->lat - endlat)*100000;
  t_yaw = atan2((cos(endlat)*(endlon - gps_msg->lon)),(endlat - gps_msg->lat))*180/pi;
  if(t_yaw < 0) t_yaw = t_yaw + 360;
  ROS_INFO("t_yaw: %f\tdisToend: %f", t_yaw,disToend);
  pre_steer = control_data.steer;
  if(times > 0 ){
	control_data.steer = prop * (t_yaw - gps_msg->yaw);
  }
  if(control_data.steer == 0){
	if(visit_flag == 1) {visit_flag = 0;}
	else{
		if(pre_steer > control_data.steer) control_data.steer = control_data.steer - add_part;
		else if(pre_steer < control_data.steer) control_data.steer = control_data.steer + add_part;
		else;
		visit_flag = 1;
	}
  }
				
  if( disToend < 1 ){
	stop_flag = 1;
  }
  if(stop_flag) {
	control_data.speed = 0;
	control_data.steer = 0;
	control_data.power = 0;
  }
  control_pub.publish(control_data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "upper_control");

  ros::NodeHandle n;
  ros::param::get("~endlat", endlat);
  ros::param::get("~endlon", endlon);
  ros::param::get("~speed", speed);
  
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  control_pub = n.advertise<driverless_test::control_data>("control_com", 1000);
  ros::Subscriber sub = n.subscribe("gps_com", 1000, controlCallback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
