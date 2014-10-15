// Include the ROS C++ APIs
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
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

const int width = 320;
const int height = 240;
TrackerMultiMarker tracker(width, height, 8, 6, 6, 6, 0);
Mat centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);
Mat cam1pos = (Mat_<float>(3,1) << 400, 0 ,0);
// Standard C++ entry point

int configTracker()
{
  tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

  // load a camera file.
  if (!tracker.init("/home/ceezeh/catkin_ws/src/more_t2/data/no_distortion.cal",
                    "/home/ceezeh/local/tools/ARToolKitPlus-2.3.0/sample/data/markerboard_480-499.cfg", 1.0f, 1000.0f)) // load MATLAB file
  {
    ROS_INFO("ERROR: init() failed\n");
    return -1;
  }

  tracker.getCamera()->printSettings();

  // the marker in the BCH test image has a thin border...
  tracker.setBorderWidth(0.125);

  // set a threshold. alternatively we could also activate automatic thresholding
  tracker.setThreshold(160);

  // let's use lookup-table undistortion for high-speed
  // note: LUT only works with images up to 1024x1024
  tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

  // switch to simple ID based markers
  // use the tool in tools/IdPatGen to generate markers
  tracker.setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
  return 0;
}

void calcPoseMatrix(TrackerMultiMarker* tracker_t, Mat* pose, Mat* T)
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker_t->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker_t->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);

  *T =Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  cout << "[Debug] T'= " << T->t() << endl;
  *pose = ((Mat)(T->t() * centreMat)).rowRange(0,3);
  *pose += cam1pos;
  cout << "pose = " << endl << *pose << endl << endl;
}
// Write pose to file.

// This functions draws a real-time 2D graph.
int cwidth = 3 * width;
int cheight = 3 * height;
Mat canvas = cvCreateImage(cvSize(cwidth, height), IPL_DEPTH_8U, 1);
Mat canvas2 = cvCreateImage(cvSize(cwidth, cheight), IPL_DEPTH_8U, 1);
void initialiseCanvas()
{
  rectangle(canvas, cvRect(0, 0, cwidth, height), 255, CV_FILLED);
  line(canvas, Point(0, height / 2), Point(cwidth, height / 2), 0);
}
void drawGraphs(Mat pose)
{
  /*
   * The idea is that each time this function is called, the image is shifted by a proportional number of pixels
   * to the left and then three separate 2D plots are generated of the trajectory for the x, y, z coordinates.
   */
  // x-axis
  static int pointer;
  // Check for wrap around.
  pointer += 5;
  if (pointer >= cwidth)
  {
    pointer = 0;
    initialiseCanvas();
  }
  float ycoord = height / 2 - pose.at<float>(1, 0) * height / 400; // For x represent 40 cm in a whole.
  cout << "ycoord" << ycoord << endl << endl;
  circle(canvas, Point(pointer, ycoord), 1.0, Scalar(0, 255, 0));
  // Use currently increment pixel points by increment frame.
  // where we assume frames move with constant interval.
  // Shift image to the left.
  imshow("Canvas", canvas);
  waitKey(1);
}

/*void rotateImage(Mat &input, Mat &output, Mat* T, float f)

{

  // get width and height for ease of use in matrices

  float w = (float)input.cols;

  float h = (float)input.rows;

  // Projection 2D -> 3D matrix

  Mat A1 = (Mat_<float>(4, 3) <<

  1, 0, -w / 2,

  0, 1, -h / 2,

  0, 0, 0,

  0, 0, 1);

  // 3D -> 2D matrix

  Mat A2 = (Mat_<float>(3, 4) <<

  f, 0, w / 2, 0,

  0, f, h / 2, 0,

  0, 0, 1, 0);

  // Final transformation matrix
  Mat R = (Mat_<float>(4,4) << (*T).at<float>(0,0), (*T).at<float>(1,0), (*T).at<float>(2,0), 0
                            , (*T).at<float>(0,1), (*T).at<float>(1,1), (*T).at<float>(2,3) ,0
                            , (*T).at<float>(0,2), (*T).at<float>(3,2), (*T).at<float>(2,2), 0
                            , 0 , 0, 0 , 1);

  Mat Trans_t = (Mat_<float>(4,4) << 1, 0 , 0, (*T).at<float>(2,0)
                                , 0, 1, 0, (*T).at<float>(2,1)
                                , 0, 0 , 1 , (*T).at<float>(2,2)
                                , 0 , 0 , 0 , 1);
  Mat trans = A2 * (Trans_t * (R * A1));

  // Apply matrix transformation

  warpPerspective(input, output, trans, input.size(), INTER_LANCZOS4);
  imshow("pose", output);
}*/

//void drawMarker(Mat* T)
//{
//  rectangle(canvas2, cvRect(0, 0, 100, 100), 255, CV_FILLED);
//
//  Mat output = cvCreateImage(cvSize(cwidth, height), IPL_DEPTH_8U, 1);
//  rotateImage(canvas2, output, T, 10);
//  imshow("pose", output);
//}

void testCSV()
{
  ofstream pFile;
  pFile.open("/media/psf/Home/Documents/PhdBase/Academic/Project/pose.csv", ios::out | ios::app);
  pFile << "1.3, " << "4.3," << "1.44,MW\n";
  pFile << "1.3, " << "4.3," << "1.44,MW\n";
  pFile.close();

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "more_t2");

  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  if (configTracker() != 0)
    return 0;
  initialiseCanvas();
  IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  IplImage *tempImg2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  IplImage* img;
  CvCapture* capture = cvCaptureFromCAM(0);
  if (argc >= 2)
  {
    capture = cvCaptureFromCAM(atoi(argv[1]));
    printf("Opening device number : %d", atoi(argv[1]));
  }
  Mat pose;
  if (!capture)
  {
    std::cout << "Could not initialize capturing...\n" << std::endl;
    return -1;
  }

  // Open data file for recording pose
  // insert here!
//	testCSV();
  // Draw marker start
//  rectangle(canvas2, cvRect(0, 0, width, height), 255, CV_FILLED);
  Mat T;
  T.create(4,4, CV_8UC(2));
  Mat poseImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  namedWindow( "pose", WINDOW_AUTOSIZE );
  while (ros::ok())
  {
    img = cvQueryFrame(capture);
    if (!img)
    {
      printf("Failed to open file");
      break;
    }

    // Crop the image to a proportion of 320 by 240.
    cvSetImageROI(img, cvRect(0, 0, 640, 480));

//		printf("Orig size %d:%d\n\n", img->width, img->height);
    IplImage *tempImg = cvCreateImage(cvGetSize(img), img->depth, 1);
    cvCvtColor(img, tempImg, CV_RGB2GRAY);
    cvResize(tempImg, tempImg2);

    cvAdaptiveThreshold(tempImg2, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 111);
//		cvThreshold(tempImg2, greyImg, (float) 70, 255.0, CV_THRESH_BINARY);
//		cvShowImage("MyVideo", greyImg);
    cvWaitKey(10); // Wait for image to be rendered on screen. If not included, no image is shown.
    int numDetected = tracker.calc((unsigned char*)greyImg->imageData);
    if (numDetected != 0)
    {
      printf("Number of Markers = %d\n\n", numDetected);
      calcPoseMatrix(&tracker, &pose, &T);
//      rotateImage(canvas2, poseImg, &T, 50);
//      imshow("pose", canvas2);
      drawGraphs(pose);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  cvReleaseCapture(&capture);
  return 0;
}
