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

double gpsYawCorrector(double gpsYaw)
{
	//if(gpsYaw > 270) return gpsYaw - 360;
	//else return gpsYaw;
	return gpsYaw;
}
double tarYawCreator(double endlat, double endlon, double gpslat, double gpslon)
{
	double t_yaw = atan2((cos(endlat)*(endlon - gpslon)),(endlat - gpslat))*180/pi;
	if(t_yaw < 0) t_yaw = t_yaw + 360;
	//if(t_yaw > 90 ) t_yaw = 180 - t_yaw;
	return t_yaw;
}
double distanceCal(double beglat, double beglon, double endlat, double endlon, double gpslat, double gpslon)
{
	double func_k = (endlat -beglat) / (endlon- beglon);//直线的斜率：y:lat x:lon
	double func_b = endlat - func_k * endlon;
	return (func_k * gpslon - gpslat + func_b) /(sqrt((func_k * func_k)+1))*100000;
}
double distance(double a1, double b1, double a2, double b2)
{
	double t = ((a1-a2)*100000);
	return sqrt(pow((a1-a2)*100000, 2) + pow((b1-b2)*100000,2));
}
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
	    /*
	    ros::param::get("~endlat", endlat);
	    ros::param::get("~endlon", endlon);
	    ros::param::get("~beglat", beglat);
	    ros::param::get("~beglon", beglon);
	    */
	    ros::param::get("~speed", speed);
	    
	    pid_init(pid_ctrl);
	    pid_set_frequency(pid_ctrl, 20);
	    
	    control_data.speed = speed;
		control_data.steer = 0;
		control_data.power = 1;
		
		count = 0;
		t_yaw = 0.0;
	    enable_flag = 0;
	    disToend = 3.0;
		
		endlat[0] = 31.88668392;
		endlon[0] = 118.80989973;
		endturn[0] = 270;
		endlat[1] = 31.88664502;
		endlon[1] = 118.80960257;
		endturn[1] = 180.0;
		endlat[2] = 33.88722013;
		endlon[2] = 120.81050525;
		endturn[2] = 270.0;
		
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
	  
	  ROS_INFO("%dendlat:%f\tendlon:%f\t", count, endlat[count], endlon[count]);
	  t_yaw = tarYawCreator(endlat[count], endlon[count], gps_msg->lat, gps_msg->lon);
	  yaw_now = gpsYawCorrector(gps_msg->yaw);
	  yaw_error = t_yaw - yaw_now;
	  if(yaw_error < 2 && yaw_error > -2) pid_reset_integral(pid_ctrl);
	  
	  
	  //dis_error = distanceCal(beglat, beglon, endlat, endlon, gps_msg->lat, gps_msg->lon);
	  //control_data.steer = pid_process(pid_ctrl, yaw_error) + 90 - 
	  //(gps_msg->lon - _endlon)*100000;
	  //control_data.steer = yaw_error*(-5.0) + 90 + dis_error* 1.0;
	  
	  
	  control_data.steer = yaw_error*(-5.0) + 90;
	  ROS_INFO("yaw_error:%f\tdis_error:%f", yaw_error, dis_error);
	  if(control_data.steer < 60) control_data.steer = 60;
	  if(control_data.steer > 120) control_data.steer = 120;
	  ROS_INFO("t_yaw: %f\tnowYaw%f\tdisToend: %f", t_yaw,yaw_now, disToend);	
	  
	  disToend = distance(gps_msg->lat, gps_msg->lon, endlat[count], endlon[count]);
	  if( disToend < 3 && !enable_flag){
		enable_flag = 1;
	  }
	  if(enable_flag){ 
	    if(fabs(gps_msg->yaw - endturn[count]) < 5) {
	      control_data.steer = 90;
		  enable_flag = 0;
	  	  count ++;
	    }
	    else{
	  	  if(endturn[count] > 0) control_data.steer = 60;
		  else control_data.steer = 120;
	    }
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
    double endlat[3];
    double endlon[3];
    double endturn[3];
    //double beglat, beglon;
    int speed;
    double yaw_error;
    double dis_error;
    double disToend;
    double yaw_now;	
    int count;
    float t_yaw;
	int enable_flag;
    
};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "upper_control");
  gpsHandler handler;
  ros::spin();

  return 0;
}
