/*
 * helper.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: parallels
 */
#include <ros/ros.h>
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>      // std::ofstream
#include <cv.h>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>
#include <math.h>
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <new>

using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

// The pose of the initial camera.
Mat centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);
TrackerMultiMarker *tracker;

struct CaptureInfo
{
  int width;
  int height;
  VideoCapture capture;
  bool newResult;
//  TrackerMultiMarker *tracker;
};
struct CameraInfo
{
  Mat camPose;
  Mat T;
  bool cameraPoseKnown;
};
struct Marker
{
  Mat marker_pose;
  int mTime;
};

void initialiseCam(int id, VideoCapture &cap)
{
  // Write a function to open camera
  // we will use an OpenCV capture
  string file = "/media/psf/Home/Documents/PhdBase/Main/Helper_Projects/More_T2/video/video_cam_";
  stringstream buffer;
  // Todo: Change to right format.
  buffer << file << id << ".mov";
//  cap.open(buffer.str().c_str());
  cap.open(id);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  printf("Opening calibration video number : %d", id);
  if (!cap.isOpened())
  {
    std::cout << "Could not initialize capturing...\n" << std::endl;
    exit(EXIT_FAILURE);
  }
}

