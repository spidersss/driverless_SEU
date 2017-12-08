#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdio.h>

#include "serial_open.h"
#include "gps_com.h"
#include "driverless_test/gps_data.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps");
  
  int fd_gps;
  //fd_gps = dev_open("/dev/ttyS0");
  if(argc != 2) {
  	std::cout<<"Error: please add the address of the device!"<<std::endl;
  	return 1;
  }
  else fd_gps = dev_open(argv[1]);

  ros::NodeHandle n;

  ros::Publisher gps_pub = n.advertise<driverless_test::gps_data>("gps_com", 1000);

  ros::Rate loop_rate(20);

  int count = 0;
  
  struct GPS_MESSAGES gps_mes;	
  char buf_gps[256];
  int len_gps = 0;
  
  while (ros::ok())
  {
    driverless_test::gps_data gps_data;
    
    len_gps = read(fd_gps, buf_gps, READ_NUM);
    gps_com(buf_gps, len_gps, &gps_mes);
    if(!gps_check(&gps_mes)) continue;			
    gps_data.id = count;
    gps_data.len = 3;
    gps_data.lat = gps_mes.lat;
    gps_data.lon = gps_mes.lon;
    gps_data.yaw = gps_mes.yaw;

    //ROS_INFO("id: %d\tlen: %d\tlat: %f\tlon: %f\tyaw: %f", gps_data.id,gps_data.len,gps_data.lat,gps_data.lon,gps_data.yaw);
	printf("%.8lf\t%.8lf\t%.1lf\n", gps_data.lat, gps_data.lon, gps_data.yaw);
   	gps_pub.publish(gps_data);
   	
    ros::spinOnce();

    loop_rate.sleep();
    
    ++count;
  }


  return 0;
}

