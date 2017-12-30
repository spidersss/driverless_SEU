#include "ros/ros.h"
#include "std_msgs/String.h"

#include "serial_open.h"
#include "driverless_test/control_data.h"

int fd_lower;
int se_buf[4];
void lowerCallback(const driverless_test::control_data::ConstPtr& control_data)
{
  ROS_INFO("steer: %d\tspeed: %d\tpower: %d", control_data->steer, control_data->speed, control_data->power);
  se_buf[0] = 255;
  se_buf[1] = control_data->steer;
  se_buf[2] = control_data->speed;
  se_buf[3] = control_data->power;
  write(fd_lower, se_buf, 4);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lower_com");

  //fd_lower = dev_open("/dev/ttyS1");
  if(argc != 2) {
  	std::cout<<"Error: please add the address of the device!"<<std::endl;
  	return 1;
  }
  else fd_lower = dev_open(argv[1]);
  ros::NodeHandle n;

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
  ros::Subscriber sub = n.subscribe("control_com", 1000, lowerCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
