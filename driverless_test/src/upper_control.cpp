#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dynamic_reconfigure/server.h"

#include <math.h>

#include "pid_controller.h"
#include "driverless_test/gps_data.h"
#include "driverless_test/control_data.h"
#include "driverless_test/driverless_Config.h"
#include "serial_node/serial_send.h"
#define pi 3.1415926
#define RAD2DEG
driverless_test::control_data control_data;
serial_node::serial_send sendmsg;
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
		send_pub = n.advertise<serial_node::serial_send>("send", 1000);
		control_sub = n.subscribe("gps_com", 1000, &gpsHandler::controlCallback, this);
		
		f = boost::bind(&gpsHandler::callbackConfig, this, _1, _2);
	    server.setCallback(f);
	    
	    ros::param::get("~endlat", endlat);
	    ros::param::get("~endlon", endlon);
	    ros::param::get("~beglat", beglat);
	    ros::param::get("~beglon", beglon);
	    ros::param::get("~speed", speed);
	    
	    pid_init(pid_ctrl);
	    pid_set_frequency(pid_ctrl, 20);
	}
	void callbackConfig(driverless_test::driverless_Config &config, uint32_t level)
	{
		ROS_INFO("Reconfigure Request: %lf %lf %lf %lf",
			config.prop,
			config.inte,
			config.diff,
			config.size);
		pid_set_gains(pid_ctrl, config.prop, config.inte, config.diff);
	}

	void controlCallback(const driverless_test::gps_data::ConstPtr& gps_msg)
	{
	  
	  ROS_INFO("endlat:%f\tendlon:%f\t", endlat, endlon);
	  if(times == 0){
		control_data.speed = speed;
		control_data.steer = 0;
		control_data.power = 1;
		func_k = (endlat -beglat) / (endlon- beglon);//直线的斜率：y:lat x:lon
	  	func_b = endlat - func_k * endlon;
		times = 1;	
	  }
	  t_yaw = atan2((cos(endlat)*(endlon - gps_msg->lon)),(endlat - gps_msg->lat))*180/pi;
	  
	  if(t_yaw < -90) t_yaw = t_yaw + 180;
	  
	  if(t_yaw > 90 ) t_yaw = 180 - t_yaw;
	  ROS_INFO("t_yaw: %f\tdisToend: %f", t_yaw,disToend);
	  pre_steer = control_data.steer;
	  //yaw_error = t_yaw - gps_msg->yaw;
	  yaw_now = gps_msg->yaw;
	  if(gps_msg->yaw > 270) yaw_now = gps_msg->yaw - 360;
	  yaw_error = t_yaw - yaw_now;
	  if(yaw_error < 2 && yaw_error > -2) pid_reset_integral(pid_ctrl);
	  dis_error = (func_k * gps_msg->lon - gps_msg->lat + func_b) /(sqrt((func_k * func_k)+1))*100000;
	  if(times > 0 ){
		//control_data.steer = pid_process(pid_ctrl, yaw_error) + 90 - 
		//(gps_msg->lon - _endlon)*100000;
		control_data.steer = yaw_error*(-5.0) + 90 + dis_error* 1.0;
		ROS_INFO("yaw_error:%f\tdis_error:%f", yaw_error, dis_error);
		if(control_data.steer < 60) control_data.steer = 60;
		if(control_data.steer > 120) control_data.steer = 120;
	  }
	  /*
	  if(control_data.steer == 0){
		if(visit_flag == 1) {visit_flag = 0;}
		else{
			if(pre_steer > control_data.steer) control_data.steer = control_data.steer - add_part;
			else if(pre_steer < control_data.steer) control_data.steer = control_data.steer + add_part;
			else;
			visit_flag = 1;
		}
	  }
	  */		
	  /*if( disToend < 1 ){
		stop_flag = 1;
	  }*/
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
	ros::Publisher send_pub;
    pid_ctrl_t* pid_ctrl = new(pid_ctrl_t);
    double endlat;
    double endlon;
    double beglat;
    double beglon;
    int speed;
    double yaw_error;
    double dis_error;
    double disToend;
    double yaw_now;	
    double func_k;
    double func_b;
    
};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "upper_control");
  gpsHandler handler;
  ros::spin();

  return 0;
}
