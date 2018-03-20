#include <ros/ros.h>
#include "serial_node/serial_receive.h"
#include "serial_node/serial_send.h"
#include "sensor_msgs/LaserScan.h"
#include "driverless_test/gps_data.h"
#include <math.h>
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)
#define R 6378137
#define speed_0 16500
#define speed_1 16530
#define pointNum 2

int bcount = 0;
bool laser_start = false;
int left = 0;
int right = 0;
float Point_Lat[pointNum]={31.888177,31.888054};
float Point_Lon[pointNum]={118.810046,118.810060};
float Lat=0;
float Lon=0;
float Yaw=0;

void receive_callback(const serial_node::serial_receive::ConstPtr& recvmsg) 
{ 
	left = recvmsg->leftspeed;
	right = recvmsg->rightspeed;
	//recv = true;
        ROS_INFO("Receive data: Leftspeed=%d, Rightspeed=%d", left,right);
	//speed += 10; 
}

//回调函数接收雷达数据,输出车辆起停状态
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float x,y,deg,degree;
	int count = scan->scan_time / scan->time_increment;
	bcount = 0;
	for(int i = 0; i < count; i++) 
	{
		deg = scan->angle_min + scan->angle_increment * i;
		degree = RAD2DEG(deg);
		x = scan->ranges[i] * sin(deg);
		y = scan->ranges[i] * cos(deg);
	 //雷达滤波范围
		if(x>=-0.35&&x<=0.35&&y>=-1.5&&y<0)
		{
			//printf("angle-situation:[%f, %f, %f, %f]\n", x,y,degree,scan->ranges[i]);
			bcount++;	
		}
	}
	if(bcount==0)
		laser_start = true;
	else
		laser_start = false;	
}
//回调函数接收GPS数据
void gpsCallback(const driverless_test::gps_data::ConstPtr& gpsend)
{
	Lat = gpsend->lat;
	Lon = gpsend->lon;
	Yaw = gpsend->yaw;
	if (Yaw>=180)
		Yaw -=360; 
	Yaw = DEG2RAD(Yaw);
}

float distance(float lat1,float lon1,float lat2, float lon2)
{
	float dis = 0;
	float Lat1 = DEG2RAD(lat1);
	float Lon1 = DEG2RAD(lon1);
	float Lat2 = DEG2RAD(lat2);
	float Lon2 = DEG2RAD(lon2);
	float a = Lat2-Lat1;
	float b = Lon2-Lon1;
	dis = 2*R*asin(sqrt(pow(sin(a/2),2)+cos(Lat1)*cos(Lat2)*pow(sin(b/2),2)));
	return dis;
}

float bearing(float lat1,float lon1,float lat2, float lon2)
{
	float bear = 0;
	float Lat1 = DEG2RAD(lat1);
	float Lon1 = DEG2RAD(lon1);
	float Lat2 = DEG2RAD(lat2);
	float Lon2 = DEG2RAD(lon2);
	float x = sin(Lon2-Lon1)*cos(Lat2);
	float y = cos(Lat1)*sin(Lat2)-sin(Lat1)*cos(Lat2)*cos(Lon2-Lon1);
	bear = atan2(x,y);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"serial_suanfa_node");  //启动该节点并设置其名称	
	ros::NodeHandle n;      //设置节点进程的句柄
	
	//订阅主题，并配置回调函数 
	ros::Subscriber recv_sub = n.subscribe("receive", 1000, receive_callback); 
	//订阅雷达主题，并配置回调函数
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
	
	//订阅雷达主题，并配置回调函数
	ros::Subscriber gps_sub = n.subscribe<driverless_test::gps_data>("gps_com", 1000, gpsCallback);
	//将节点设置成发布者，并将所发布主题和类型的名称告知节点管理器
	ros::Publisher send_pub = n.advertise<serial_node::serial_send>("send", 1000); 	
	
	//发送数据的频率为10Hz
	ros::Rate loop_rate(20);
	int i=0;
	float bear = 0;
	float yawerr = 0;
	float str = 0;
	float dis = 0;
	bool gps_start = true;
	float speed = speed_0;
	while(ros::ok())
	{
	//当收到停止消息或者ROS停止当前节点运行时,ros::ok()行会执行停止节点运行的命令
		dis = distance(Lat,Lon,Point_Lat[i],Point_Lon[i]);
		if(dis<=1.0)
		{
			i++;
		}
		if(i==pointNum)
		{
			gps_start = false;
			i=0;
		}
		bear = bearing(Lat,Lon,Point_Lat[i],Point_Lon[i]);
		yawerr = bear-Yaw;
		str = atan(sin(yawerr)/2.5f);
		serial_node::serial_send sendmsg;
		if(laser_start == true&&gps_start == true)
		{
			if (speed<speed_1)
				speed++;
		}
		else
		{
			speed = speed_0;
		}
		sendmsg.speed = speed;
		short temp = 0;
		temp = (short)(27*RAD2DEG(str));
		if(temp<-550)
		{
			temp = -550;
		}
		if(temp>550)
		{
			temp = 550;
		}
		sendmsg.Steer = temp+16450;
		send_pub.publish(sendmsg);  //消息被发布
		ros::spinOnce();  //如果有一个订阅者出现，ROS就会更新并读取所有主题
		loop_rate.sleep();  //按照10Hz的频率将程序挂起
	}
	return 0;
}
