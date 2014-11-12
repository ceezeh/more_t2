// Grab.cpp
/*
 Note: Before getting started, Basler recommends reading the Programmer's Guide topic
 in the pylon C++ API documentation that gets installed with pylon.
 If you are upgrading to a higher major version of pylon, Basler also
 strongly recommends reading the Migration topic in the pylon C++ API documentation.

 This sample illustrates how to grab and process images using the CInstantCamera class.
 The images are grabbed and processed asynchronously, i.e.,
 while the application is processing a buffer, the acquisition of the next buffer is done
 in parallel.

 The CInstantCamera class uses a pool of buffers to retrieve image data
 from the camera device. Once a buffer is filled and ready,
 the buffer can be retrieved from the camera object for processing. The buffer
 and additional image data are collected in a grab result. The grab result is
 held by a smart pointer after retrieval. The buffer is automatically reused
 when explicitly released or when the smart pointer object is destroyed.
 */
#include <ros/ros.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <pylon/PylonImage.h>
#include <pylon/Pixel.h>
#include <pylon/ImageFormatConverter.h>

using namespace cv;
// Namespace for using cout.
using namespace std;

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "grab");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  namedWindow("CV_Image", WINDOW_AUTOSIZE);
  VideoCapture cap("/home/ceezeh/catkin_ws/src/more_t2/video/video_cam_0.mov");
  if (!cap.isOpened()) {
    std::cout << "Could not initialize capturing...\n" << std::endl;
          exit(EXIT_FAILURE);
  }
  Mat frame;
  while (ros::ok())
  {
    cap >> frame;

      imshow("CV_Image", frame);
      waitKey(1);
    ros::spinOnce();

    loop_rate.sleep();
  }

}

