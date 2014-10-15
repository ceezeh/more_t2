/*
 * camera_pose_calibration.cpp
 *
 *  Created on: 15 Oct 2014
 *      Author: ceezeh
 */


#include <ros/ros.h>

int main(int argc, char** argv)
{
  int id =0;
  ros::init(argc, argv, "cam1");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);


  n.getParam("id",id );

  while (ros::ok())
   {
    printf("id: %d \n",id);
    ros::spinOnce();
    loop_rate.sleep();
   }
}
