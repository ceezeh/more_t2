/*
 * camera_pose_calibration.cpp
 *
 *  Created on: 15 Oct 2014
 *      Author: ceezeh
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
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;
using namespace cv;
// Namespace for using pylon objects.
using namespace Pylon;
using ARToolKitPlus::TrackerMultiMarker;

int id = 1;
int width;
int height;
bool cameraPoseKnown = false;
CvCapture* capture = NULL;
TrackerMultiMarker *tracker;
ros::Publisher markerPose_pub;
Mat cam_pose;
// The pose of the initial camera.
Mat marker_pose;
Mat centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);

//Pylon variables.
// Automagically call PylonInitialize and PylonTerminate to ensure
// the pylon runtime system is initialized during the lifetime of this object.
Pylon::PylonAutoInitTerm autoInitTerm;
CInstantCamera camera;
CGrabResultPtr ptrGrabResult;
//Debug variable
bool camInitialise = false;
bool newMarker = false;



 ros::Subscriber markerPose_sub;

//bool isCalibrated()
//{
//
//  if (id == 0)
//  {
//    return true;
//  }
//  ifstream file;
//  stringstream sbuffer;
//  string filepath = "../cfg/cfg_";
//  sbuffer << filepath << id;
//  file.open((const string)(sbuffer.str()), ios::in);
//  if (!file.is_open())
//  {
//    printf("Error! Camera not yet Calibrated");
//    return false;
//  }
//  /*
//   * Configuration data is of the form:
//   * ip:
//   * poseData:
//   */
//  file.close();
//  return true;
//}

// Fail program if cam is not initialised
void initialiseCam(int deviceNum = 0)
{
  // temp solution
  if (id == 0)
  { // we will use an OpenCV capture
    capture = cvCaptureFromCAM(deviceNum);
    printf("Opening device number : %d", deviceNum);
    if (!capture)
    {
      std::cout << "Could not initialize capturing...\n" << std::endl;
      exit (EXIT_FAILURE);
    }
    camInitialise = true;
  }
  else
  {
    try
    {
      // assign camera no to a full name.
      CTlFactory& tlFactory = CTlFactory::GetInstance();

      // Get all attached devices and exit application if no device is found.
      DeviceInfoList_t devices;
      if (tlFactory.EnumerateDevices(devices) == 0)
      {
        throw RUNTIME_EXCEPTION( "No camera present.");
        exit (EXIT_FAILURE);
      }

      // Todo change this to correct id value
      camera.Attach(tlFactory.CreateDevice(devices[id - 1]));

      // Print the model name of the camera.
      cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
      camInitialise = true;
      // Start camera grabbing
      camera.StartGrabbing(GrabStrategy_LatestImageOnly);
    }
    catch (GenICam::GenericException &e)
    {
      // Error handling
      cerr << "An exception occurred." << endl << e.GetDescription() << endl;
      exit(EXIT_FAILURE);
    }
  }

}

void initFrameSize()
{
  // Write a function to open camera
  if (id == 0)
  {
    IplImage *img = cvQueryFrame(capture);
    if (!img)
    {
      printf("Failed to open image from capture device");
      exit(EXIT_FAILURE);
    }
    width = img->width;
    height = img->height;
  }
  else
  {
    camera.RetrieveResult(500, ptrGrabResult, TimeoutHandling_ThrowException);
    if (ptrGrabResult->GrabSucceeded())
    {
      width = ptrGrabResult->GetWidth();
      height = ptrGrabResult->GetHeight();
      cout << "width" << width << endl;
    }
  }
}

int configTracker()
{

  // write a function get camera frame size.
  tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
  tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

  // load a camera file.
  if (!tracker->init("/home/ceezeh/catkin_ws/src/more_t2/data/no_distortion.cal",
                     "/home/ceezeh/local/tools/ARToolKitPlus-2.3.0/sample/data/markerboard_480-499.cfg", 1.0f, 1000.0f)) // load MATLAB file
  {
    ROS_INFO("ERROR: init() failed\n");
    exit(EXIT_FAILURE);
  }

  tracker->getCamera()->printSettings();

  // the marker in the BCH test image has a thin border...
  tracker->setBorderWidth(0.125);

  // set a threshold. alternatively we could also activate automatic thresholding
  tracker->setThreshold(160);

  // let's use lookup-table undistortion for high-speed
  // note: LUT only works with images up to 1024x1024
  tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

  // switch to simple ID based markers
  // use the tool in tools/IdPatGen to generate markers
  tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
  return 0;
}

void initialisation()
{
  // Initialise Capturing Device.

  initialiseCam(id);
  initFrameSize();
  configTracker();
}

// Updates and publishes marker pose.
void publishMarkerPose()
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);
  Mat T = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
      cout << "[Debug] T'= " << endl << T.t() << endl << endl;

  cout << "[Debug 2] T'= " << endl << T.t() << endl << endl;
  Mat pose_t = ((Mat)(T.t() * centreMat)).rowRange(0, 3);
//  pose_t += cam_pose;
  cout << "pose = " << endl << pose_t << endl << endl;

  geometry_msgs::Vector3Stamped pose;
  pose.header.stamp = ros::Time().now();
  pose.vector.x = pose_t.at<float>(0, 0);
  pose.vector.y = pose_t.at<float>(0, 1);
  pose.vector.z = pose_t.at<float>(0, 2);
  markerPose_pub.publish(pose);
}

// Get transformation matrix from camera to marker.
// Returns true if a new transformation matrix was obtained.
bool processMarkerImg(IplImage *img)
{
  int numDetected = 0;
  IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  if (id == 0)
  {
    IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
    IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    cvCvtColor(img, tempImg, CV_RGB2GRAY);
    cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 111);
    numDetected = tracker->calc((unsigned char*)greyImg->imageData);
  }
//  cvShowImage("Proof2", greyImg);
//  cvWaitKey(10); // Wait for image to be rendered on screen. If not included, no image is shown.
  else
  {
    cvAdaptiveThreshold(img, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 111);
    numDetected = tracker->calc((unsigned char*)greyImg->imageData);
  }

  if (numDetected != 0)
  {
    printf("Number of Markers = %d\n\n", numDetected);

    return true;
  }
  return false;
}

void getCameraPose(Mat* marker_pose_t)
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);
  Mat T = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  cout << "[Debug]: T = " << endl << T << endl << endl;
  Mat T_t = (Mat(((Mat)(T.inv())).t()));
  cout << "[Debug]: T_t = " << endl << T_t << endl << endl;
  Mat mpose_t;
  mpose_t.create(4,1, CV_32FC1);
  mpose_t.at<float>(0,0) = marker_pose_t->at<float>(0,0);
  mpose_t.at<float>(1,0) = marker_pose_t->at<float>(0,0);
  mpose_t.at<float>(2,0) = marker_pose_t->at<float>(0,0);
  mpose_t.at<float>(3,0) = 1;
  cam_pose = (Mat(T_t * mpose_t)).rowRange(0, 3);;
  cout << "[Debug] cam_pose'= " << endl << cam_pose << endl << endl;
  cameraPoseKnown = true;
  newMarker = false;
}

void markerPoseCallback(const geometry_msgs::Vector3Stamped & msg)
{
  ros::Duration timeout(0, 100000000); // 100 milli second.
  if ((ros::Time::now() - msg.header.stamp) < timeout)
  {
    marker_pose.at<float>(0, 0) = msg.vector.x;
    marker_pose.at<float>(0, 1) = msg.vector.y;
    marker_pose.at<float>(0, 2) = msg.vector.z;
    newMarker = true;
    cout << "New marker!" << endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam1");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  bool pubsubToggle = false;

  n.getParam("id", id);
  markerPose_sub = n.subscribe("/global/markerpose", 2, markerPoseCallback);
  Mat T;
  T.create(4,4, CV_32FC1);
  marker_pose.create(3,1, CV_32FC1);;
  //Pylon stuff

  // Initialise all global variables.
  initialisation();

  // Get original camera pose.
  if (id == 0)
  {
    stringstream stream;
    string pose_s;
    int x, y, z;
    n.getParam("pose", pose_s);
    stream << pose_s;
    stream >> x >> y >> z;
    cam_pose = (Mat_<float>(3, 1) << x, y, z);
    cameraPoseKnown = true;
  }

  IplImage* img = NULL;
  if (id != 0)
  {
    img = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
  }
  while (ros::ok())
  {
    // Debug prints.
//    printf("id: %d \n", id);
    if (id == 0) // Only try to output marker's global pose. No need to try to get camera's pose.
    {
      img = cvQueryFrame(capture);
      if (!img)
      {
        ROS_INFO("ERROR: files to grab new image\n");
        exit(EXIT_FAILURE);
      }
    }
    else
    { // Use a pylon interface.
      camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
      if (ptrGrabResult->GrabSucceeded())
      {
        cvSetData(img, (uint8_t*)ptrGrabResult->GetBuffer(), ptrGrabResult->GetWidth());
        cvShowImage("Proof2", img);
        cvWaitKey(10); // Wait for image to be rendered on screen. If not included, no image is shown.
      } else {
        ROS_INFO("ERROR: files to grab new image\n");
                exit(EXIT_FAILURE);
      }
    }

    bool newResult = processMarkerImg(img); // Get transformation matrix from new result.

    if (camInitialise)
    {
      printf("Cam:%d initialised\n\n", id);
    }
    if (cameraPoseKnown)
    {
      // Update PubSub
      // Shutdown subcriber if still on and start publisher if not started
      if (pubsubToggle == false) {
        markerPose_pub = n.advertise<geometry_msgs::Vector3Stamped>("/global/markerpose", 2);
        markerPose_sub.shutdown();
        pubsubToggle = true;
      }

      // If camera's position is known then start publishing marker position
      // publish marker pose.
//      if (id == 0)
      cout << "cam_pose" << endl << cam_pose << endl << endl;
      if (newResult)
        publishMarkerPose();
    }
    else
    { // Derive Camera's position from marker image.
      cout << "Unknown Cam Pose" << endl;
      if (newResult && newMarker){
        cout << "Getting cam pose" << endl;
        getCameraPose( &marker_pose);
      }
    }
    // Update pub sub

    // If this camera is calibrated, then send the position of the marker as ROS message.
    // To do this, first tracker marker and report its position.

    // If camera's pose is not known then we need to check for new marker positions.

      ros::spinOnce();

    loop_rate.sleep();
  }
}
