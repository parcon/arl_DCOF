#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <geometry_msgs/Vector3.h>


#define EV_KEY          0x01
#define EV_REL          0x02
#define EV_ABS          0x03

int dx,dy,dz;

void mouse_cb(const geometry_msgs::Vector3& msg)
{
  msg.x=dx;
  msg.y=dy;
  msg.z=dz;
}


int main(int argc, char** argv)
{
  int fd;
  ros::init (argc, argv, "Mouse Control");
  ros::Rate loop_rate(500);
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("mouse_control", 10, mouse_cb)



fd = open("/dev/input/uinput", O_WRONLY | O_NONBLOCK);
if(fd < 0) {
    ROS_ERROR("Mouse uinput error 1");
    exit(0);

if(ioctl(fd, UI_SET_EVBIT, EV_REL) < 0)
       ROS_ERROR("Mouse uinput error 2");
    if(ioctl(fd, UI_SET_RELBIT, REL_X) < 0)
        ROS_ERROR("Mouse uinput error 2");
    if(ioctl(fd, UI_SET_RELBIT, REL_Y) < 0)
        ROS_ERROR("Mouse uinput error 2");
        
memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "DCOF-mouse_control");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1;
    uidev.id.product = 0x1;
    uidev.id.version = 1;

if(write(fd, &uidev, sizeof(uidev)) < 0)
        ROS_ERROR("Mouse uinput error 3");

    if(ioctl(fd, UI_DEV_CREATE) < 0)
       ROS_ERROR("Mouse uinput error 3");

    sleep(2);
    
    while (node.ok() && ros::ok()) {//while ROS and node are working

 memset(&ev, 0, sizeof(struct input_event));
ev.type = EV_REL;
ev.code = REL_X;
ev.value = dx;
if(write(fd, &ev, sizeof(struct input_event)) < 0)
	ROS_ERROR("Mouse dx");

memset(&ev, 0, sizeof(struct input_event));
ev.type = EV_REL;
ev.code = REL_Y;
ev.value = dy;
if(write(fd, &ev, sizeof(struct input_event)) < 0)
	ROS_ERROR("Mouse dy");

/*
memset(&ev, 0, sizeof(struct input_event));
ev.type = EV_SYN;
ev.code = 0;
ev.value = 0;
if(write(fd, &ev, sizeof(struct input_event)) < 0)
ROS_ERROR("Mouse syn");
*/
ros::spinOnce ();
  loop_rate.sleep ();

}//while ros ok
ret = ioctl(fd, UI_DEV_DESTROY); //destroys mouse input
ROS_ERROR("Mouse control or ROS crashed");
}//main