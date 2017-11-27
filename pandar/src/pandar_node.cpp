/*
 *  Copyright (c) 2017, Hesai Photonics Technology Co., Ltd
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  
 *  The views and conclusions contained in the software and documentation are those
 *  of the authors and should not be interpreted as representing official policies,
 *  either expressed or implied, of the FreeBSD Project.
 */

#include "pandar_grabber/pandar_grabber.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/io.h>
#include <ros/ros.h>

class SimpleGrabber
{
public:
	SimpleGrabber(ros::NodeHandle& nh);
	~SimpleGrabber();
	void run();

private:
	ros::NodeHandle& nh_;
	std::string ip_;
	std::string frame_id_;
	std::string pcap_;
	int port_;

	void cloud_callback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud);
	boost::shared_ptr<pcl::PandarGrabber> grabber_;

	ros::Publisher cloud_pub_;
};

int main(int argc , char** argv)
{
    ros::init(argc, argv, "pandar_node");
    ros::NodeHandle nh("~");
    SimpleGrabber grb(nh);
	grb.run();
    ros::spin();
    return 0;
}

SimpleGrabber::SimpleGrabber(ros::NodeHandle& nh):
	nh_(nh)
{
	nh_.param<std::string>("device_ip", ip_, "");
	nh_.param<std::string>("frame_id", frame_id_, "pandar");
	nh_.param<int>("port", port_, 2368);
	nh_.param<std::string>("pcap", pcap_, "");
	if (ip_ != "") {
		ROS_INFO_STREAM("grabber listening on " << ip_ << " port " << port_);
		grabber_.reset(new pcl::PandarGrabber(boost::asio::ip::address::from_string(ip_),
					port_, ""));
	} else if (pcap_ != "") {
		ROS_INFO_STREAM("no ip address spicified, using pcap...");
		ROS_INFO_STREAM(pcap_);
		grabber_.reset(new pcl::PandarGrabber("", pcap_));
	} else {
		ROS_ERROR_STREAM("no ip address spicified, no pcap file provided, what can I do for you...");
	}
//	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
	cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("pandar_points", 10);
}

SimpleGrabber::~SimpleGrabber()
{
	if (grabber_)
		grabber_->stop();
}

void SimpleGrabber::cloud_callback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud)
{
	//publish
	ROS_INFO_STREAM("got a cloud. size " << cloud->size());
	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
	points->points.resize(cloud->size());
	for (int i = 0; i < cloud->size(); i++) {
		const pcl::PointXYZI& spt = cloud->points[i];
		pcl::PointXYZI& dpt = points->points[i];
		dpt.x = spt.x;
		dpt.y = spt.y;
		dpt.z = spt.z;
		dpt.intensity = spt.intensity;
	}
	points->header.frame_id = frame_id_;
	points->header.stamp = cloud->header.stamp & 0x000FFFFFFFFFFFFF;
	points->height = 1;
	points->width = points->size();
	cloud_pub_.publish(points);
}

void SimpleGrabber::run()
{
	boost::function<pcl::PandarGrabber::sig_cb_Hesai_Pandar_sweep_point_cloud_xyzi> f =
		boost::bind(&SimpleGrabber::cloud_callback, this, _1);
	boost::signals2::connection c = grabber_->registerCallback(f);
	grabber_->start();
}
