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
#include <std_msgs/String.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

int width;
int height;
TrackerMultiMarker *tracker;
Mat centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);
int sema_signals = 0;
Mat cam_pose;
int configTracker(int width_t, int height_t)
{
  tracker = new TrackerMultiMarker(width_t, height_t, 8, 6, 6, 6, 0);
  tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

  // load a camera file.
  if (!tracker->init("/home/ceezeh/catkin_ws/src/more_t2/data/no_distortion.cal",
                     "/home/ceezeh/local/tools/ARToolKitPlus-2.3.0/sample/data/markerboard_480-499.cfg", 1.0f, 1000.0f)) // load MATLAB file
  {
    ROS_INFO("ERROR: init() failed\n");
    return -1;
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

void calcMarkerPose(TrackerMultiMarker* tracker_t, Mat* pose)
{
  // Todo: Change this to detect other markers
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker_t->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker_t->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);

  Mat T = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  cout << "[Debug] T'= " << T.t() << endl;
  // Update Translation part of pose of marker from camera
  Mat subpose = pose->rowRange(0, 3);
  Mat xyz = ((Mat)(T.t() * centreMat)).rowRange(0, 3);
  xyz.copyTo(subpose);

  // Calculate orientation.

  // Construct a rotation matrix from global to camera frame.
  // First construct rotation matrix from global to camera frame
  float cosA = cos(cam_pose.at<float>(3,0)), sinA= sin(cam_pose.at<float>(3,0)),
      cosB = cos(cam_pose.at<float>(4,0)), sinB= sin(cam_pose.at<float>(4,0)),
      cosC = cos(cam_pose.at<float>(5,0)), sinC= sin(cam_pose.at<float>(5,0));
  Mat Rz = (Mat_<float>(3,3) << cosC,-sinC, 0,
                            sinC, cosC, 0,
                             0, 0 , 1);
  Mat Rx = (Mat_<float>(3,3) << 1, 0, 0,
                            0, cosA, -sinA,
                             0, sinA , cosA);
  Mat Ry = (Mat_<float>(3,3) << cosB, 0, sinB,
                              0, 1, 0,
                            -sinB, 0, cosB);
  Mat Rg2c = Rz*Rx*Ry;
  Mat Rg2m = T.rowRange(0,3).colRange(0,3) * Rg2c;

  // Update Orientation Part of Pose of marker from camera.
  // heading = atan2(-r20,r00)
  // Roll A
  pose->at<float>(3, 0) = asin(Rg2m.at<float>(2, 1));
  // Yaw B
  pose->at<float>(4, 0) = atan2(-Rg2m.at<float>(2, 0), Rg2m.at<float>(2, 2));
  // Pitch C
  pose->at<float>(5, 0) = atan2(-Rg2m.at<float>(0, 1), Rg2m.at<float>(1, 1));

  cout << "pose = " << endl << *pose << endl << endl;
}

void getCamPose(Mat * poseArray, int index)
{
  ifstream file;
  stringstream sbuffer;
  string filepath = "/home/ceezeh/catkin_ws/src/more_t2/cfg/cfg_";
  sbuffer << filepath << index;
  file.open(sbuffer.str().c_str(), ios::in);
  if (!file.is_open())
  {
    cout << "Error! Cannot open file to save Calibration" << endl;
    exit(EXIT_FAILURE);
  }
  char data[100];
  file.getline(data, 100);
  float x, y, z, roll, yaw, pitch;
  file.getline(data, 100);
  x = atof(data);
  file.getline(data, 100);
  y = atof(data);
  file.getline(data, 100);
  z = atof(data);
  file.getline(data, 100);
  roll = atof(data);
  file.getline(data, 100);
  yaw = atof(data);
  file.getline(data, 100);
  pitch = atof(data);
  file.close();
  *poseArray = (Mat_<float>(6, 1) << x, y, z, roll, yaw, pitch);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "more_t2");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);

  int noCams = 2;
  n.getParam("noCams", noCams); //The total number of possible cameras.
  // Generate cam_pose array based on number of cameras and referenced by ID;

  // Generate tracking information for all relevant camera feeds.
  for (int id = 0; id < noCams; id++)
  {

    // Initialise capturing device.
    // We assume frames are all of the same size and are all gray scale.
    stringstream sbuffer;
    string filepath = "/home/ceezeh/catkin_ws/src/more_t2/video/video_cam_";
    sbuffer << filepath << id << ".mov";

//    CvCapture* capture = cvCaptureFromAVI(sbuffer.str().c_str());
    CvCapture* capture = cvCaptureFromCAM(id);
    sbuffer.str("");
    if (!capture)
    {
      std::cout << "Could not initialize capturing...\n" << std::endl;
      exit(EXIT_FAILURE);
    }
    //Initiliase width and heigth.
    IplImage* img;
    img = cvQueryFrame(capture);
    if (!img)
    {
      cout << "Error! Cannot open capture object." << endl;
      exit(EXIT_FAILURE);
    }
    width = img->width;
    height = img->height;

    // Get the camera's pose

    getCamPose(&cam_pose, id);

    if (configTracker(width, height) != 0)
    {
      cout << "Error! Cannot configure tracker." << endl;
      exit(EXIT_FAILURE);
    }
    IplImage *tempImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    Mat pose;
    pose.create(6, 1, CV_32F);
    Mat T;
    T.create(4, 4, CV_8UC(2));

    Mat poseImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    float t = 0; // Todo: Correct for time variable.

    string path = "/home/ceezeh/catkin_ws/src/more_t2/posedata/pose_cam_";
    sbuffer << path << id << ".csv";
    cout << "sbuffer" << endl << sbuffer.str() << endl;
    cout << "id" << endl << id << endl;
    // File stream to save data.
    ofstream fp;
    fp.open(sbuffer.str().c_str(), ios::out | ios::trunc);
    sbuffer.str("");
    if (!fp.is_open())
    {
      cout << "Error! Cannot open file to save pose data" << endl;
      exit(EXIT_FAILURE);
    }
    // Todo: While video is not empty
    while (ros::ok())
    {
      cvCvtColor(img, tempImg, CV_RGB2GRAY);
      cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 111);


      int numDetected = tracker->calc((unsigned char*)greyImg->imageData);
      char name[10];
      sprintf(name, "%d", id);
      cvShowImage(name, greyImg);
      cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.
      if (numDetected != 0)
      {
        printf("Number of Markers = %d\n\n", numDetected);
        calcMarkerPose(tracker, &pose);
        // Todo: Implement Constant frame rate!
        // Save pose to file with  time stamp.
        // To do this, increment each time based on frame rate.
        // Here we assume frame rate is constant as given by a hardware trigger.

        //We save each row in the form, x, y, z, t;
        // Todo: Save information on 6DOF trajectory so include:
        // Yaw, Pitch and Row information.
        fp << pose.at<float>(0, 0) << "," << pose.at<float>(1, 0) << "," << pose.at<float>(2, 0) << ","
            << pose.at<float>(3, 0) << "," << pose.at<float>(4, 0) << "," << pose.at<float>(5, 0) << "," << t << "\n";
      }
      // Todo: Scale time by frame rate factor.
      t++;
      if (t >=500){
        break;
      }
      img = cvQueryFrame(capture);
      if (!img)
      {
        printf("End of File!");
        break;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
    cvReleaseCapture(&capture);
    fp.close();
  }

  return 0;
}
