基于pandar grabber的ROS包
---
这个包在ROS中创建一个节点，用于从以太网中读取禾赛Pandar 40线激光雷达的数据，亦可用于从pcap文件中读取离线保存的雷达数据。
其中**pandar grabber**路径下的程序可以单独应用于其他C++程序。

## 依赖
1. libpcap-dev  
  On Ubuntu, `sudo apt-get install libpcap-dev`
2. pcl >= 1.7  
3. protocal buffer  
  On Ubuntu, `sudo apt-get install protobuf-compiler libprotobuf-dev`

## 编译
```
catkin_make
```

## 运行
从以太网中读取实时雷达数据
```
roslaunch pandar_ros pandar.launch device_ip:=<device ip> port:=<port>
```
从pcap文件中读取雷达数据
```
roslaunch pandar_ros pandar.launch pcap:=<full path to pcap file>
```

### 发布的消息 
* /pandar\_node/pandar\_points [sensor\_msgs/PointCloud2]

## 测试过的环境
* ROS indigo on Ubuntu14.04
* ROS kinetic on Ubuntu16.04
