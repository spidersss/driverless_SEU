#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include "serial_node/serial_receive.h"
#include "serial_node/serial_send.h"

#define u8 unsigned char 
#define recvNum 2
#define sendNum 2
#define recvsize 7
#define sendsize 9
 
u8 recvbuff[recvsize];
u8 sendbuff[sendsize];

short send_data[sendNum];
short recv_data[recvNum];
serial::Serial ser; //声明串口对象
//接收数据处理函数
short bytetodate(u8 buf[2])
{
	short temp;
	temp = buf[0];
	temp<<=8;
	temp += buf[1];
	return temp;
}

void recvmessage(u8* recvbuff,short* date,int num)
{
	int i=0;
	u8 buf[2];
	u8 fuhao; 
	if(recvbuff[0]==0xff&&recvbuff[1]==0xff)
	{
		fuhao = recvbuff[2];
		for(i=0;i<num;i++)
		{
			buf[0]=recvbuff[2*i+3];
			buf[1]=recvbuff[2*i+4];
			date[i]=bytetodate(buf);
			if((fuhao&1)==1)
				date[i]*=-1;
			fuhao>>=1;
		}
	}
}
//发送数据处理函数
void datetobyte(short date,u8 buf[2])
{
	short temp;
	temp = date;
	buf[1]=temp;  //buf[1]记录低8位
	temp>>=8;
	buf[0]=temp;  //buf[0]记录高8位
}

void sendmessage(u8* sendbuff,short* date,int num)
{
	int i=0;
	short temp;
	u8 buf[2];
	sendbuff[0]=0xff;
	sendbuff[1]=0xff;
	sendbuff[2]=0x00;
	for(i=0;i<num;i++)
	{
		temp = date[i];
		if(temp<0)
		{
			sendbuff[2]|=1<<i;
			temp*=-1;
		} 
		datetobyte(temp,buf);
		sendbuff[2*i+3]=buf[0];
		sendbuff[2*i+4]=buf[1];
	}
	i=i-1;
	sendbuff[2*i+5]= '\r';
	sendbuff[2*i+6]= '\n';
}
//发送回调函数 
void send_callback(const serial_node::serial_send::ConstPtr& sendmsg) 
{ 
	//int i = 0;
	send_data[0] = sendmsg->speed;
	send_data[1] = sendmsg->Steer;
       	sendmessage(sendbuff,send_data,sendNum);
	ser.write(sendbuff,sendsize);   //发送串口数据
} 
 
int main (int argc, char** argv) 
{ 
	int i = 0;
	for(i=0;i<recvNum;i++)
		recv_data[i] = 0;
	for(i=0;i<sendNum;i++)
		send_data[i] = 0;	
	//初始化节点 
	ros::init(argc, argv, "serial_example_node"); 
	//声明节点句柄 
	ros::NodeHandle nh; 

	//订阅主题，并配置回调函数 
	ros::Subscriber write_sub = nh.subscribe("send", 1000, send_callback); 
	//发布主题 
	ros::Publisher read_pub = nh.advertise<serial_node::serial_receive>("receive", 1000); 

	try 
	{ 
		//设置串口属性，并打开串口 
		ser.setPort("/dev/stm32"); 
		ser.setBaudrate(115200); 
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
		ser.setTimeout(to); 
		ser.open(); 
	 }
	catch (serial::IOException& e) 
	{ 
		ROS_ERROR_STREAM("Unable to open port "); 
		return -1; 
	} 

	//检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()) 
	{ 
		ROS_INFO_STREAM("Serial Port initialized"); 
	} 
	else 
	{ 
		return -1; 
	} 

	//指定循环的频率 
	ros::Rate loop_rate(10); 
	while(ros::ok()) 
	{ 
		if(ser.available())
		{ 
			ROS_INFO_STREAM("Reading from serial port");  
			ser.read(recvbuff,recvsize);
			/*
			for(i=0;i<recvsize;i++)
			{
			 	ROS_INFO("[0x%02x]",recvbuff[i]);
			}*/
			recvmessage(recvbuff,recv_data,recvNum);
			ROS_INFO(" Leftspeed=%d, rightspeed=%d",recv_data[0],recv_data[1]);
			
			//sendmessage(sendbuff,send_data,sendNum);
			//for(i=0;i<sendsize;i++)
			//{
			 	//ROS_INFO("[0x%02x]",sendbuff[i]);
			//}
			//ser.write(sendbuff,sendsize);   //发送串口数据 
			//使用自定义接收消息包
			serial_node::serial_receive recvmsg;
			recvmsg.leftspeed = recv_data[0];
			recvmsg.rightspeed = recv_data[1]; 
			read_pub.publish(recvmsg);   //发布消息
		} 
		//处理ROS的信息，比如订阅消息,并调用回调函数 
		ros::spinOnce(); 
		loop_rate.sleep(); 
	} 
	return 0;
} 
