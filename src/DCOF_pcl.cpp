/*
Parker Conroy
Richard Kirby
ARLab @ University of Utah
For Advanced Mechatronics (ME6240) Semester Project

This code takes serial data over the wire from the DCOF and build a PCL point cloud for the purposes of visualization and data processing

It depends on:
PCL_ros
Cereal_port 
&&
target_link_libraries(DCOF_pcl cereal_port) in Cmake
*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cereal_port/CerealPort.h>


#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_SPEED 57000
#define REPLY_SIZE 8
#define TIMEOUT 1000

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char** argv)
{
  ros::init (argc, argv, "publish_DCOFS");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<PointCloud> ("custom_laser_scaner", 1);

  cereal::CerealPort device;
  char serial_data[REPLY_SIZE];
  char * pEnd;
  long int enc1, enc2, dist;

  try{ device.open(SERIAL_PORT, SERIAL_SPEED); }
  catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");


  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "Laser_Frame";
  msg->height = msg->width = 1;
  

  ros::Rate loop_rate(4);
  while (node.ok() && ros::ok())//while ROS and node are working
  {
        try{ device.read(serial_data, REPLY_SIZE, TIMEOUT); } // Get the reply, the last value is the timeout in ms
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }
        ROS_INFO("Serial Message: %s", serial_data);
/*
Extract dist and encoders from serial message

*/ 
  enc1 = strtol (serial_data,&pEnd,10);
  enc2 = strtol (pEnd,&pEnd,10);
  dist = strtol (pEnd,NULL,10);
   


/*
This section will include the math that takes the encoder values and distance from the DCOF 
and outputs the point in x,y,z

as encoders proportinal to angle
*/
y=dist*sin(theta_y);
x=dist*sin(theta_x);
z=dist*cos(theta_y);

  
   msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
   msg->header.stamp = ros::Time::now ();
   pub.publish (msg);

  ros::spinOnce ();
  loop_rate.sleep ();
}//while
}//main

