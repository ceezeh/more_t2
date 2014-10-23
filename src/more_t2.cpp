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

void calcPoseMatrix(TrackerMultiMarker* tracker_t, Mat* pose)
{
  // Todo: Change this to detect other markers
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker_t->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker_t->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);

   Mat T = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  cout << "[Debug] T'= " << T.t() << endl;
  // Update x, y, z part of pose
  Mat subpose = pose->rowRange(0,2);
  Mat xyz = ((Mat)(T.t() * centreMat)).rowRange(0, 3);
  xyz.copyTo(subpose);

  // Update Orientation Part of Pose.
  // heading = atan2(-r20,r00)
  pose->at<float>(3,0) = atan2(-T.at<float>(2,0),T.at<float>(0,0));
  // bank = atan2(-r12,r11)
  pose->at<float>(4,0) = atan2(-T.at<float>(1,2), T.at<float>(1,1));
  // attitude = asin(r10)
  pose->at<float>(5,0) = asin(T.at<float>(1,0));

  *pose += cam_pose;
  cout << "pose = " << endl << *pose << endl << endl;
}

void getCamPose(Mat * poseArray, int index)
{
  ifstream file;
  stringstream sbuffer;
  string filepath = "/home/ceezeh/catkin_ws/src/more_t2/cfg/cfg_";
  sbuffer << filepath << index;
  file.open((const string)(sbuffer.str()), ios::in);
  if (!file.is_open())
  {
    cout << "Error! Cannot open file to save Calibration" << endl;
    exit(EXIT_FAILURE);
  }
  char data[100];
  file.getline(data, 100);
  float x, y, z;
  file.getline(data, 100);
  x = atof(data);
  file.getline(data, 100);
  y = atof(data);
  file.getline(data, 100);
  z = atof(data);
  file.close();
  *poseArray = (Mat_<float>(3, 1) << x, y, z);

}
void semaCallback(const std_msgs::String::ConstPtr& msg)
{
  sema_signals++;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "more_t2");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);

  int noOfCams = 0;
  int maxNoOfCams = 6;
  n.getParam("max_no_of_cams", maxNoOfCams); //The total number of possible cameras.
  // Generate cam_pose array based on number of cameras and referenced by ID;
//  Mat cam_poses[maxNoOfCams];
  bool activeCams[maxNoOfCams];

  // Create a boolean of available cameras.
  // Also find number of active cameras.
  int temp = 0;
  stringstream sbuffer;
  for (int i = 0; i < maxNoOfCams; i++)
  {
    sbuffer << "cam" << i;
    n.getParam(sbuffer.str().c_str(), temp);
    if (temp == 0)
    {
      activeCams[i] = false;
    }
    else
    {
      activeCams[i] = true;
      noOfCams++;
    }
    sbuffer.str("");
  }
// For test!!
//  activeCams[0] = true;
//  activeCams[1] = true;
//  noOfCams = 2;
  // Wait for all recorder node to signal to our semaphore.
  ros::Subscriber sema_sub = n.subscribe("/global/sema", noOfCams, semaCallback);
  while (sema_signals != 1) // replace with noOfCams
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  sema_sub.shutdown();


int id = -1;
  // Generate tracking information for all relevant camera feeds.
  for (int itr = 0; itr < noOfCams; itr++)
  {

    // Get the next camera's id.

  while (activeCams[++id] == false) {
    if (id >= maxNoOfCams) {
      cout << "End of Video Feeds!." << endl;
                        exit(EXIT_FAILURE);
    }
  }



    // Initialise capturing device.
    // We assume frames are all of the same size and are all gray scale.
      string filepath = "/home/ceezeh/catkin_ws/src/more_t2/videos/video_cam_";
      sbuffer << filepath << id <<".avi";

    CvCapture* capture = cvCaptureFromAVI(sbuffer.str().c_str());
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

    if (configTracker(width, height) != 0){
      cout << "Error! Cannot configure tracker." << endl;
                        exit(EXIT_FAILURE);
    }
    IplImage *tempImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    Mat pose;
    pose.create(6,1, CV_32F);
    Mat T;
    T.create(4, 4, CV_8UC(2));

    Mat poseImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    float t = 0; // time variable.

    string path = "/home/ceezeh/catkin_ws/src/more_t2/posedata/pose_cam_";
    sbuffer << path << id << ".csv";
    cout << "sbuffer" << endl<< sbuffer.str() << endl;
    cout << "id" << endl <<id<< endl;
    // File stream to save data.
    ofstream fp;
    fp.open((const string)(sbuffer.str()), ios::out|ios::trunc);
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

      cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.
      int numDetected = tracker->calc((unsigned char*)greyImg->imageData);
      if (numDetected != 0)
      {
        printf("Number of Markers = %d\n\n", numDetected);
        calcPoseMatrix(tracker, &pose);
        // Todo: Implement Constant frame rate!
        // Save pose to file with  time stamp.
        // To do this, increment each time based on frame rate.
        // Here we assume frame rate is constant as given by a hardware trigger.


        //We save each row in the form, x, y, z, t;
        // Todo: Save information on 6DOF trajectory so include:
        // Yaw, Pitch and Row information.
          fp << pose.at<float>(0,0) <<"," << pose.at<float>(1,0) << ","<< pose.at<float>(2,0) << ","<<t<<"\n";
      }
      // Todo: Scale time by frame rate factor.
      t++;
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
