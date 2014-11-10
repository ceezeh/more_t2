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
#include <cv.h>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <new>          // ::operator new[]

using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

const ros::Duration timeout(0, 100000000); // 100 milli second.
int noCams = 1;
int* widthArr;
int* heightArr;
bool* camerasPoseKnown;
VideoCapture* captureArr;
TrackerMultiMarker **trackerArr;
Mat *camPoseArr;
// The pose of the initial camera.
Mat centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);

//Debug variable
bool camInitialise = false;
bool *newResultArr;
//Todo: Change this to frame captured time.
ros::Time arrivalTime;
struct Marker{
  Mat marker_pose;
  ros::Time mTime;
};
Marker marker;

void initialiseCam(int id, VideoCapture cap)
{
  // Write a function to open camera
  // we will use an OpenCV capture
  string file = "home/ceezeh/catkin_ws/src/more_t2/cfg/cfg_vid_";
  stringstream buffer;
  // Todo: Change to right format.
  buffer << file << id << ".avi";
  cap.open(buffer.str().c_str());
  printf("Opening calibration video number : %d", id);
  if (!captureArr[id].isOpened())
  {
    std::cout << "Could not initialize capturing...\n" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void initFrameSize(VideoCapture cap, int width, int height)
{
  // Write a function to open camera
  Mat frame;
  IplImage *img;
  cap >> frame;
  img = cvCreateImage(cvSize(frame.cols, frame.rows), IPL_DEPTH_8U, frame.channels());
  img->imageData = (char *)frame.data;
  if (frame.empty())
  {
    printf("Failed to open image from capture device");
    exit(EXIT_FAILURE);
  }
  width = img->width;
  height = img->height;
}

int configTracker(TrackerMultiMarker * t, int width, int height)
{
  // write a function get camera frame size.
  t = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
  t->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

  // load a camera file.
  if (!t->init("/home/ceezeh/catkin_ws/src/more_t2/data/logitech.cal",
                     "/home/ceezeh/local/tools/ARToolKitPlus-2.3.0/sample/data/markerboard_480-499.cfg", 1.0f, 1000.0f)) // load MATLAB file
  {
    ROS_INFO("ERROR: init() failed\n");
    exit(EXIT_FAILURE);
  }

  t->getCamera()->printSettings();

  // the marker in the BCH test image has a thin border...
  t->setBorderWidth(0.125);

  // set a threshold. alternatively we could also activate automatic thresholding
  t->setThreshold(130);

  // let's use lookup-table undistortion for high-speed
  // note: LUT only works with images up to 1024x1024
  t->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

  // switch to simple ID based markers
  // use the tool in tools/IdPatGen to generate markers
  t->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
  return 0;
}

//void initialisation(int id, TrackerMultiMarker * t, VideoCapture cap)
//{
//  // Initialise Capturing Device.
//
//  initialiseCam(id);
//  initFrameSize(cap, width, height);
//  configTracker(t);
//}

// Updates and publishes marker pose.
void updateMarkerPose(TrackerMultiMarker* tracker, Mat cam_pose)
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);
  Mat T = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  // Fixing the z translation value.
//  if (id == 1) { if (id == 1) {
//  T.at<float>(3,2) *=1.6;
//}
  cout << "[Debug] T'= " << endl << T.t() << endl << endl;
  Mat pose_t;
  pose_t.create(6, 1, CV_32F);
  // Update Translation part of pose of marker from camera
  Mat subpose = pose_t.rowRange(0, 3);
  Mat xyz = ((Mat)(T.t() * centreMat)).rowRange(0, 3);

  xyz.copyTo(subpose);

  // Translation of marker from Global Frame
  subpose += cam_pose.rowRange(0, 3);
  // Calculate orientation.

  // Construct a rotation matrix from global to camera frame.
  // First construct rotation matrix from global to camera frame
  float cosA = cos(cam_pose.at<float>(3, 0)), sinA = sin(cam_pose.at<float>(3, 0)), cosB = cos(
      cam_pose.at<float>(4, 0)),
        sinB = sin(cam_pose.at<float>(4, 0)), cosC = cos(cam_pose.at<float>(5, 0)), sinC = sin(
            cam_pose.at<float>(5, 0));
  Mat Rz = (Mat_<float>(3, 3) << cosC, -sinC, 0, sinC, cosC, 0, 0, 0, 1);
  Mat Rx = (Mat_<float>(3, 3) << 1, 0, 0, 0, cosA, -sinA, 0, sinA, cosA);
  Mat Ry = (Mat_<float>(3, 3) << cosB, 0, sinB, 0, 1, 0, -sinB, 0, cosB);
  Mat Rg2c = Rz * Rx * Ry;
  Mat Rg2m = T.rowRange(0, 3).colRange(0, 3) * Rg2c;

  // Update Orientation Part of Pose of marker from camera.
  // heading = atan2(-r20,r00)
  // Roll A
  pose_t.at<float>(3, 0) = asin(Rg2m.at<float>(2, 1));
  // Yaw B
  pose_t.at<float>(4, 0) = atan2(-Rg2m.at<float>(2, 0), Rg2m.at<float>(2, 2));
  // Pitch C
  pose_t.at<float>(5, 0) = atan2(-Rg2m.at<float>(0, 1), Rg2m.at<float>(1, 1));
  pose_t.copyTo(marker.marker_pose);
  marker.mTime = ros::Time::now();
}

// Get transformation matrix from camera to marker.
// Returns true if a new transformation matrix was obtained.
bool processMarkerImg(IplImage *img, TrackerMultiMarker *tracker, int width, int height)
{
  int numDetected = 0;
  IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

  IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
  cvCvtColor(img, tempImg, CV_RGB2GRAY);
  cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 111);
  numDetected = tracker->calc((unsigned char*)greyImg->imageData);
  cvShowImage("Proof2", greyImg);
  cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.

  if (numDetected != 0)
  {
    printf("Number of Markers = %d\n\n", numDetected);

    return true;
  }
  return false;
}

void getCameraPose(TrackerMultiMarker* tracker, Mat* marker_pose_t, Mat* cam_pose)
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);

  Mat T = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  // Temporary Fix for the z translation value.
//  if (id == 1) {
//    T.at<float>(3,2) *=1.6;
//  }

  cout << "[Debug]: T' = " << endl << T.t() << endl << endl;
  Mat Tc2m = ((Mat)(T.t() * centreMat)).rowRange(0, 3);
  cout << "[Debug]: Tc2m' = " << endl << Tc2m << endl << endl;

  // Calculate cam's translational position
  Mat temp = marker_pose_t->rowRange(0, 3) - Tc2m;
  cout << "[Debug]: temp' = " << endl << temp << endl << endl;
  Mat cam_trans = cam_pose->rowRange(0, 3);
  temp.copyTo(cam_trans);
  cout << "[Debug]: cam_trans' = " << endl << cam_trans << endl << endl;
  // Calculate cam's orientation pose.
  Mat Rc2m = T.rowRange(0, 3).colRange(0, 3);
  cout << "[Debug]: Rc2m' = " << endl << Rc2m << endl << endl;
  // Rg2m has to be reconstructed from marker orientation

  float cosA = cos(marker_pose_t->at<float>(3, 0)), sinA = sin(marker_pose_t->at<float>(3, 0)), cosB = cos(
      marker_pose_t->at<float>(4, 0)),
        sinB = sin(marker_pose_t->at<float>(4, 0)), cosC = cos(marker_pose_t->at<float>(5, 0)), sinC = sin(
            marker_pose_t->at<float>(5, 0));
  Mat Rz = (Mat_<float>(3, 3) << cosC, -sinC, 0, sinC, cosC, 0, 0, 0, 1);
  Mat Rx = (Mat_<float>(3, 3) << 1, 0, 0, 0, cosA, -sinA, 0, sinA, cosA);
  Mat Ry = (Mat_<float>(3, 3) << cosB, 0, sinB, 0, 1, 0, -sinB, 0, cosB);
  Mat Rg2m = Rz * Rx * Ry;

  Mat Rg2c = Rc2m.inv() * Rg2m;
  // Roll A
  cam_pose->at<float>(3, 0) = asin(Rg2c.at<float>(2, 1));
  // Yaw B
  cam_pose->at<float>(4, 0) = atan2(-Rg2c.at<float>(2, 0), Rg2c.at<float>(2, 2));
  // Pitch C
  cam_pose->at<float>(5, 0) = atan2(-Rg2c.at<float>(0, 1), Rg2c.at<float>(1, 1));

  cout << "[Debug] cam_pose'= " << endl << cam_pose << endl << endl;
}

void writeCalibrationToFile(int id, Mat cam_pose)
{
  // Here we save the id, device fullname and camera pose
  ofstream file;
  stringstream sbuffer;
  string filepath = "/home/ceezeh/catkin_ws/src/more_t2/cfg/cfg_";
  sbuffer << filepath << id;
  file.open(sbuffer.str().c_str(), ios::out | ios::trunc);
  if (!file.is_open())
  {
    cout << "Error! Cannot open file named\" " << sbuffer.str() << " \" to save Calibration" << endl;
    exit(EXIT_FAILURE);
  }
  /*
   * Configuration data is of the form:
   * ip:
   * poseData:
   */

  cout << "Writing to file!" << endl;
  file << cam_pose.at<float>(0, 0) << "\n";  // x
  file << cam_pose.at<float>(1, 0) << "\n";  // y
  file << cam_pose.at<float>(2, 0) << "\n";  // z
  file << cam_pose.at<float>(3, 0) << "\n";  // roll
  file << cam_pose.at<float>(4, 0) << "\n";  // yaw
  file << cam_pose.at<float>(5, 0) << "\n";  // pitch
  file.close();
  cout << "Written to file!" << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam1");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  bool pubsubToggle = false;

  n.getParam("noCams", noCams);

  Mat T;
  T.create(4, 4, CV_32FC1);
  marker.marker_pose = Mat::zeros(6, 1, CV_32F);

  // Initialise all global variables.
  widthArr = new int[noCams];
  heightArr = new int[noCams];
  captureArr = new VideoCapture[noCams];
  trackerArr = new TrackerMultiMarker*[noCams];
  newResultArr = new bool[noCams];
  for (int i = 0; i < noCams; i++)
  {
    camerasPoseKnown[i] = false;
    initialiseCam(i,captureArr[i]);
    camPoseArr[i].create(6, 1, CV_32F);
    initFrameSize(captureArr[i], widthArr[i], heightArr[i]);
    configTracker(trackerArr[i], widthArr[i], heightArr[i]);
    newResultArr[i] = false;
  }

// Initialise known camera's position.
  stringstream stream;
  string pose_s;
  int x, y, z, roll, yaw, pitch;
  n.getParam("pose", pose_s);
  stream << pose_s;
  stream >> x >> y >> z >> roll >> yaw >> pitch;
  camPoseArr[0] = (Mat_<float>(6, 1) << x, y, z, roll, yaw, pitch);
  camerasPoseKnown[0] = true;
  writeCalibrationToFile(0, camPoseArr[0]);

  IplImage* imgArr[noCams];

  for (int id = 0; id < noCams; id++)
  {
    Mat temp;
    captureArr[id].read(temp);
    imgArr[id] = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U, temp.channels());
  }

  while (ros::ok())
  {

    for (int id = 0; id < noCams; id++)
    {

      Mat temp;
      captureArr[id].read(temp);
      int count = captureArr[id].get(CV_CAP_PROP_POS_MSEC);
      cout << "Attention!! " << count << endl;
      if (temp.empty())
      {
        ROS_INFO("ERROR: files to grab new image\n");
        exit(EXIT_FAILURE);
      }

      imgArr[id]->imageData = (char*)temp.data;

      newResultArr[id] = processMarkerImg(imgArr[id],trackerArr[id], widthArr[id], heightArr[id]); // Get transformation matrix from new result.

      if (camerasPoseKnown[id])
      {
        // Update PubSub
        // Shutdown subscriber if still on and start publisher if not started
        if (pubsubToggle == false)
        {
          pubsubToggle = true;
        }

        // If camera's position is known then start publishing marker position
        // publish marker pose.
        cout << "cam_pose" << endl << camPoseArr[id] << endl << endl;
        if (newResultArr[id])
          updateMarkerPose(trackerArr[id], camPoseArr[id]);
          newResultArr[id] = false;
      }
      else
      { // Derive Camera's position from marker image.
        cout << "Unknown Cam Pose" << endl;

        if (newResultArr[id] && ((ros::Time::now() - marker.mTime) < timeout))
        {
          cout << "Getting cam pose" << endl;
          getCameraPose(trackerArr[id], &marker.marker_pose, &camPoseArr[id]);
          camerasPoseKnown[id] = true;
          newResultArr[id] = false;

          writeCalibrationToFile(id, camPoseArr[id]);
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
}
