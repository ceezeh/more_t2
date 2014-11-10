/*
 * grab.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: ceezeh
 */

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
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>      // std::ofstream
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#include <pylon/PylonGUI.h>
#endif
#include <pylon/PylonImage.h>
#include <pylon/Pixel.h>
#include <pylon/ImageFormatConverter.h>
#include <cv.h>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>
#include <math.h>
#include <std_msgs/String.h>

using namespace cv;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;
char camera_name[100];
Mat cam_pose;
CInstantCamera camera;
VideoCapture capture;
bool camInitialise = false;
int width;
int height;
int id = 0;
CGrabResultPtr ptrGrabResult;
// Opens camera device by id and gets the camera's position.
void getConfig()
{
  ifstream file;
  stringstream sbuffer;
  string filepath = "/home/ceezeh/catkin_ws/src/more_t2/cfg/cfg_";
  sbuffer << filepath << id;
  file.open(sbuffer.str().c_str(), ios::in);
  if (!file.is_open())
  {
    cout << "Error! Cannot open file to save Calibration" << endl;
    exit(EXIT_FAILURE);
  }
  char data[100];
  file.getline(camera_name, 100);
  float x, y, z;
  file.getline(data, 100);
  x = atof(data);
  file.getline(data, 100);
  y = atof(data);
  file.getline(data, 100);
  z = atof(data);
  file.close();
  cam_pose = (Mat_<float>(3, 1) << x, y, z);
}

void initialiseCam(int deviceNum = 0)
{
  // Write a function to open camera
  // we will use an OpenCV capture
     capture.open(deviceNum);
     capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
     capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
     printf("Opening device number : %d", deviceNum);
     if (!capture.isOpened())
     {
       std::cout << "Could not initialize capturing...\n" << std::endl;
       exit (EXIT_FAILURE);
     }
     camInitialise = true;
}

void initFrameSize()
{
    // Write a function to open camera
     Mat frame;
       IplImage *img;
       capture>>frame;
       img = cvCreateImage(cvSize(frame.cols,frame.rows), IPL_DEPTH_8U, frame.channels());
       img->imageData = (char *)frame.data;
       if (frame.empty())
       {
         printf("Failed to open image from capture device");
         exit(EXIT_FAILURE);
       }
       width = img->width;
       height = img->height;
}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "recorder");

  ros::NodeHandle n("~");

  ros::Rate loop_rate(10);

  n.getParam("id", id);

  ros::Publisher sema_pub = n.advertise<std_msgs::String>("/global/sema", 1);
// The exit code of the sample application.
  getConfig();
  initialiseCam();
  initFrameSize();
// Automagically call PylonInitialize and PylonTerminate to ensure
// the pylon runtime system is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;

  CGrabResultPtr ptrGrabResult;


  stringstream sbuffer;
  string videoPath = "/home/ceezeh/catkin_ws/src/more_t2/videos/video_cam_";
    sbuffer << videoPath << id << ".avi";

    Size frameSize(static_cast<int>(width), static_cast<int>(height));
    // Todo: Get precise frame rate. This static value is very wrong.
    Mat temp;
            capture.read(temp);
            IplImage* img  = cvCreateImage(cvSize(temp.cols,temp.rows), IPL_DEPTH_8U, temp.channels());

  CvVideoWriter* oVideoWriter = cvCreateVideoWriter(sbuffer.str().c_str(), CV_FOURCC_DEFAULT, 5, cvSize(width, height), 0);
  if ( !oVideoWriter ) //if not initialize the VideoWriter successfully, exit the program
    {
         cout << "ERROR: Failed to start video writer" << endl;
         exit(EXIT_FAILURE);
    }

  while(ros::ok()) {
    capture.read(temp);
    if (temp.empty())
            {
              ROS_INFO("ERROR: files to grab new image\n");
              exit(EXIT_FAILURE);
            }

    img->imageData = (char*)temp.data;
            cvWriteToAVI(oVideoWriter,img);
            cvShowImage("Debug", img);
            cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.

          cout << "cam pose" << endl << cam_pose << endl << endl;

      ros::spinOnce();
      loop_rate.sleep();
    }
  cvReleaseVideoWriter(&oVideoWriter);
  std_msgs::String msg;
  msg.data = "Arrived";
  sema_pub.publish(msg);
  return 0;
}
