/*
Parker Conroy
Richard Kirby
ARLab @ University of Utah
For Advanced Mechatronics (ME6960) Semester Project

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
#include <geometry_msgs/Vector3.h>

#define SERIAL_PORT "/dev/ttyUSB0"
//#define SERIAL_SPEED 57000
#define SERIAL_SPEED 57600
#define REPLY_SIZE 500
#define TIMEOUT 100


//reply_size 57
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
float deg2rad= 0.0174532925;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "publish_DCOFS");
  
  ros::NodeHandle node;
  ros::Publisher cloud_pub = node.advertise<PointCloud> ("custom_laser_scaner", 1);
  ros::Publisher mouse_pub = node.advertise<geometry_msgs::Vector3> ("mouse_commands", 1);
  ros::Rate loop_rate(500);
  
  
  cereal::CerealPort device;
  char serial_data[REPLY_SIZE];
  char * pEnd;
  double theta_y, theta_x, x, y, z;
  double x_old, y_old, z_old;
  x_old=y_old=z_old= 0.0;

double dist, x1,y1,squal1, x2, y2, squal2, pwm1, pwm2, enc1, enc2; //X1;Y1;SQUAL1;X2;Y2;SQUAL2;PWM1;PWM2;QEI1;QEI2
float enc1_max=45000;//encoder counts from one side to another
float enc2_max=45000;
float degree_x=70;//degrees of throw
float degree_y=120;
  geometry_msgs::Vector3 mouse_msg;
/*
tf::TransformBroadcaster br;
 tf::Transform desiredtf;

 desiredVec.setX(0.0);
      desiredVec.setY(0.0);
      desiredVec.setZ(2.0);
	desiredtf.setOrigin( desiredVec );
    desiredtf.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );
br.sendTransform( tf::StampedTransform(desiredtf, ros::Time::now(), "/optitrak", "/desired"));

*/
  try{ device.open(SERIAL_PORT, SERIAL_SPEED); }
  catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");


	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "/map";
//  msg->height = msg->width = 1;

  while (node.ok() && ros::ok())//while ROS and node are working
  {
        try{ device.read(serial_data, REPLY_SIZE, TIMEOUT); } // Get the reply, the last value is the timeout in ms
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Serial Timeout!");
        }
     ROS_INFO("Serial Message: %s", serial_data);

		//Extract data and encoders from serial message
		//X1;Y1;SQUAL1;X2;Y2;SQUAL2;PWM1;PWM2;QEI1;QEI2
		x1 = strtol (serial_data,&pEnd,10);
		y1 = strtol (pEnd,&pEnd,10);
		squal1 = strtol (pEnd,&pEnd,10);
		x2 = strtol (pEnd,&pEnd,10);
		y2 = strtol (pEnd,&pEnd,10);
		squal2 = strtol (pEnd,&pEnd,10);
		pwm1 = strtol (pEnd,&pEnd,10);
		pwm2 = strtol (pEnd,&pEnd,10);
		enc1 = strtol (pEnd,&pEnd,10);
		enc2 = strtol (pEnd,NULL,10);
	//	dist = strtol (pEnd,NULL,10);

//ROS_INFO("Decoded Message: %lf %lf", enc1,enc2);

		//Tri for where point is
	//	dist=x1-x2*y1-y2; //NEEDS fixing
		theta_x=(enc1-enc1_max/2)*(degree_x/enc1_max);
		theta_y=(enc2-enc2_max/2)*(degree_y/enc2_max);
		
		y=1.0*sin(deg2rad*theta_y);
		x=1.0*sin(deg2rad*theta_x);
		//z=1.0*cos(deg2rad*theta_y);

ROS_INFO("POINT: %lf %lf ", x,y);
		

	ROS_INFO("Gimbal orientation~ x:%f	y: %f", theta_x,theta_y);
	//ROS_INFO("Hand Position~ x:%f	y:%f	z:%f", x, y, z);
	//ROS_INFO("Mouse Movements~ x:%f	y:%f	z:%f", x1, y1, z-z_old);
		
		msg->points.assign(1,pcl::PointXYZ(x, y, z));
		msg->header.stamp = ros::Time::now ();
		cloud_pub.publish (msg);

		mouse_msg.x=x-x_old;
		mouse_msg.y=y-y_old;
		mouse_msg.z=z-z_old;
		mouse_pub.publish(mouse_msg);


		y=y_old;
		x=x_old;
		//z=z_old;

		ros::spinOnce ();
		loop_rate.sleep ();
}//while
}//main

