#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dynamic_reconfigure/server.h"

#include <math.h>

#include "driverless_test/gps_data.h"
#include "driverless_test/control_data.h"
#include "driverless_test/driverless_Config.h"
#define pi 3.1415926

driverless_test::control_data control_data;
double disToend = 0.0;	
float t_yaw = 0.0;
int stop_flag = 0;
int visit_flag = 0;
float add_part = 25;
float pre_steer = 90;
int times = 0;
class gpsHandler
{
public:
	gpsHandler()
	{
		control_pub = n.advertise<driverless_test::control_data>("control_com", 1000);
		control_sub = n.subscribe("gps_com", 1000, &gpsHandler::controlCallback, this);
		
		f = boost::bind(&gpsHandler::callbackConfig, this, _1, _2);
	    server.setCallback(f);
	    
	    ros::param::get("~endlat", endlat);
	    ros::param::get("~endlon", endlon);
	    ros::param::get("~speed", speed);
	}
	void callbackConfig(driverless_test::driverless_Config &config, uint32_t level)
	{
		ROS_INFO("Reconfigure Request: %lf %lf %lf %lf",
			config.prop,
			config.inte,
			config.diff,
			config.size);
		prop = config.prop;
		inte = config.inte;
		diff = config.diff;
	}

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
protected:
	ros::NodeHandle n;
	ros::Publisher control_pub;
	ros::Subscriber control_sub;
	dynamic_reconfigure::Server<driverless_test::driverless_Config> server;
    dynamic_reconfigure::Server<driverless_test::driverless_Config>::CallbackType f;
    double prop;
    double inte;
    double diff;
    double endlat;
    double endlon;
    int speed;
};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "upper_control");
  gpsHandler handler;
  ros::spin();

  return 0;
}
