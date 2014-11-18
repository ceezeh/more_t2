/*
 * camera_pose_calibration.cpp
 *
 *  Created on: 15 Oct 2014
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
#include <new>          // ::operator new[]

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

void initFrameSize(VideoCapture &cap, int &width, int &height)
{
  // Write a function to open camera
  Mat frame;
  IplImage *img;
  cap.read(frame);
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

int configTracker(int width, int height)
{
  // write a function get camera frame size.
  tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
  tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

  // load a camera file.
  if (!tracker->init("/home/parallels/catkin_ws/src/more_t2/data/logitech_new.cal",
                     "/home/parallels/tools/ARToolKitPlus-2.3.1/sample/data/markerboard_480-499.cfg", 1.0f, 1000.0f)) // load MATLAB file
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

//void initialisation(int id, TrackerMultiMarker * t, VideoCapture cap)
//{
//  // Initialise Capturing Device.
//
//  initialiseCam(id);
//  initFrameSize(cap, width, height);
//  configTracker(t);
//}

void getRotMatrix(float roll, float yaw, float pitch, Mat &R) {
	 float cosA = cos(roll), sinA = sin(roll), cosB = cos(yaw),
	        sinB = sin(yaw), cosC = cos(pitch), sinC = sin(pitch);
	  Mat Rz = (Mat_<float>(3, 3) << cosC, -sinC, 0, sinC, cosC, 0, 0, 0, 1);
	  Mat Rx = (Mat_<float>(3, 3) << 1, 0, 0, 0, cosA, -sinA, 0, sinA, cosA);
	  Mat Ry = (Mat_<float>(3, 3) << cosB, 0, sinB, 0, 1, 0, -sinB, 0, cosB);
	  Mat temp = Rz * Rx * Ry;
	  // Todo: Notice the transformation.
	  ((Mat)temp.t()).copyTo(R);
}
// Updates and publishes marker pose.
void updateMarkerPose(TrackerMultiMarker* tracker, Mat cam_pose, Marker marker, Mat &T)
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);
  Mat tmp = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  tmp.copyTo(T);

  Mat pose_t;
  pose_t.create(6, 1, CV_32F);


  // Calculate orientation.

  // Construct a rotation matrix from global to camera frame.
  // First construct rotation matrix from global to camera frame
  Mat Rc2g;
  Rc2g.create(3,3, CV_32F);
  getRotMatrix(cam_pose.at<float>(3, 0), cam_pose.at<float>(4, 0), cam_pose.at<float>(5, 0), Rc2g);
  Mat Rm2c = T.rowRange(0, 3).colRange(0, 3).t();
  Mat Rm2g = Rc2g * Rm2c;

  // Update Orientation Part of Pose of marker from camera.
  // heading = atan2(-r20,r00)
  // Roll A
  pose_t.at<float>(3, 0) = asin(Rm2g.at<float>(1, 2));
  // Yaw B Todo: I remove the minus sign on the first element
  pose_t.at<float>(4, 0) = atan2(-Rm2g.at<float>(0, 2), Rm2g.at<float>(2, 2));
  // Pitch C
  pose_t.at<float>(5, 0) = atan2(-Rm2g.at<float>(1, 0), Rm2g.at<float>(1, 1));
  pose_t.copyTo(marker.marker_pose);
//  marker.mTime = ros::Time::now();


  Mat cPose = (Mat_<float>(4, 1) << 0, 0, 0, 1);
    Mat marker_t = ((Mat)(T.t() * cPose));

    Mat Trans_t;
    Trans_t.create(3,4, CV_32F);
    hconcat(Rc2g, cam_pose.rowRange(0,3), Trans_t);
    Mat Trans;
    Trans.create(4,4, CV_32F);
    Mat off = (Mat_<float>(1, 4) << 0,0,0,1);
    vconcat(Trans_t,off , Trans);

    // Calculate Translation
    Mat m = ((Mat)(Trans * marker_t)).rowRange(0,3);
    Mat mTrans = marker.marker_pose.rowRange(0, 3);
    m.copyTo(mTrans);
    cout << "marker trans = " << endl << mTrans << endl << endl;

}

// Get transformation matrix from camera to marker.
// Returns true if a new transformation matrix was obtained.
bool processMarkerImg(IplImage *img, TrackerMultiMarker *tracker, int width, int height, int id)
{
  int numDetected = 0;
  IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

  IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
  cvCvtColor(img, tempImg, CV_RGB2GRAY);
  cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 111);
  numDetected = tracker->calc((unsigned char*)greyImg->imageData);
  char name[10];
  sprintf(name,"%d",id);
  cvShowImage(name, greyImg);
  cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.

  if (numDetected != 0)
  {
//    printf("Number of Markers = %d\n\n", numDetected);

    return true;
  }
  return false;
}

void getCameraPose(TrackerMultiMarker* tracker, Mat* marker_pose_t, Mat* cam_pose, Mat&T)
{
  ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
  ARFloat nOpenGLMatrix[16];
  ARFloat markerWidth = 102;
  ARFloat patternCentre[2] = {0.0f, 0.0f};
  tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth, nOpenGLMatrix);

  Mat tmp = Mat(4, 4, CV_32FC1, (float *)nOpenGLMatrix);
  tmp.copyTo(T);
//  cout << "[Debug]: T' = " << endl << T.t() << endl << endl;
  Mat Tc2m = ((Mat)(T.t().inv() * centreMat)).rowRange(0, 3);

//  cout << "[Debug]: temp' = " << endl << temp << endl << endl;
  Mat cam_trans = cam_pose->rowRange(0, 3);


//  cout << "[Debug]: cam_trans' = " << endl << cam_trans << endl << endl;
  // Calculate cam's orientation pose.

//  cout << "[Debug]: Rc2m' = " << endl << Rc2m << endl << endl;
  // Rg2m has to be reconstructed from marker orientation
  Mat Rc2m = T.rowRange(0, 3).colRange(0, 3);
  Mat Rm2g;
  Rm2g.create(3,3,CV_32F);
  getRotMatrix(marker_pose_t->at<float>(3, 0),marker_pose_t->at<float>(4, 0), marker_pose_t->at<float>(5, 0), Rm2g);
  Mat Rc2g = Rm2g*Rc2m;
  // Roll A
  cam_pose->at<float>(3, 0) = asin(Rc2g.at<float>(1, 2));
  // Yaw B
  cam_pose->at<float>(4, 0) = atan2(-Rc2g.at<float>(0, 2), Rc2g.at<float>(2, 2));
  // Pitch C
  cam_pose->at<float>(5, 0) = atan2(-Rc2g.at<float>(1, 0), Rc2g.at<float>(1, 1));

  // Translation
  // first get the camera's position in terms of the marker coordinates
  // Reconstruct transformation matrix from world to marker
  Mat mPose = (Mat_<float>(4, 1) << 0,0,0, 1);
  Mat cam_t = ((Mat)(T.t().inv() * mPose));
  // Then transforms from the marker frame to the global frame.
  // Frame reconstruct the transformation matrix

  Mat Trans_t;
  Trans_t.create(3,4, CV_32F);
  hconcat(Rm2g, marker_pose_t->rowRange(0,3), Trans_t);
  Mat Trans;
  Trans.create(4,4, CV_32F);
  Mat off = (Mat_<float>(1, 4) << 0,0,0,1);
  vconcat(Trans_t,off , Trans);
  //  cout << "trans" << endl << Trans << endl << endl << "marker pose" << endl<< *marker_pose_t << endl<< endl<< "Rg2m" << endl << Rg2m<< endl << endl;

  Mat cam = ((Mat)(Trans * cam_t)).rowRange(0,3);
  cam.copyTo(cam_trans);
//  cout << "[Debug] cam_pose'= " << endl << cam_pose << endl << endl;
}

void writeCalibrationToFile(int id, Mat cam_pose)
{
  // Here we save the id, device fullname and camera pose
  ofstream file;
  stringstream sbuffer;
  string filepath = "/home/parallels/catkin_ws/src/more_t2/cfg/cfg_";
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

//  cout << "Writing to file!" << endl;
  file << cam_pose.at<float>(0, 0) << "\n";  // x
  file << cam_pose.at<float>(1, 0) << "\n";  // y
  file << cam_pose.at<float>(2, 0) << "\n";  // z
  file << cam_pose.at<float>(3, 0) << "\n";  // roll
  file << cam_pose.at<float>(4, 0) << "\n";  // yaw
  file << cam_pose.at<float>(5, 0) << "\n";  // pitch
  file.close();
//  cout << "Written to file!" << endl;
}

int cwidth  = 5*320;
int height = 800;
Mat canvas = cvCreateImage(cvSize(cwidth, height), IPL_DEPTH_32F, 3);

void initialiseTrans() {
	rectangle(canvas, cvRect(0,0, cwidth, height), Scalar(255, 255, 255), CV_FILLED );
	line( canvas, Point( 0, 100 ), Point( cwidth, 100), 0,4);
	line( canvas, Point( 0, 300 ), Point( cwidth, 300), 0,4 );
	line( canvas, Point( 0, 800 ), Point( cwidth, 800), 0,4 );

	int csize = 200;
	// Create Grid lines.
	int step = csize/20;
	for (int i = 0; i < csize; i += step) {
		line( canvas, Point( 0, i), Point( cwidth, i), 0 );
		char text[10];
		sprintf(text, "%d", csize/2 - i);
		putText(canvas,(const char*)text, Point(0,i),FONT_HERSHEY_PLAIN, 1,Scalar(0,0,255) );
	}
	for (int i = csize; i < 2*csize; i += step) {
		line( canvas, Point( 0, i), Point( cwidth, i), 0 );
		char text[10];
		sprintf(text, "%1.0f", 1.5*csize - i);
		putText(canvas,(const char*)text, Point(0,i),FONT_HERSHEY_PLAIN, 1,Scalar(0,0,255) );
		}
	csize*=2;
	for (int i = csize; i < 2*csize; i += step) {
		line( canvas, Point( 0, i), Point( cwidth, i), 0 );
		char text[10];
		sprintf(text, "%1.0d",  (2*csize-i)*2);
		putText(canvas,(const char*)text, Point(0,i),FONT_HERSHEY_PLAIN, 1,Scalar(0,0,255) );
		}
}

void drawTrans(Mat trans) {
	/*
	 * The idea is that each time this function is called, the image is shifted by a proportional number of pixels
	 * to the left and then three separate 2D plots are generated of the trajectory for the x, y, z coordinates.
	 */
	// x-axis
	static int pointer;
	// Check for wrap around.
	pointer += 5;
	if (pointer >= cwidth) {
		pointer = 0;
		initialiseTrans();
	}
	float coord1 = 100 - trans.at<float>(0,0); // For x represent 40 cm in a whole.
	float coord2 = 300 - trans.at<float>(0,1); // For x represent 40 cm in a whole.
	float coord3 = 800 - trans.at<float>(0,2)/2; // For x represent 40 cm in a whole.
	circle( canvas, Point( pointer, coord1), 1.0, Scalar( 0, 255, 0 ) );
	circle( canvas, Point( pointer, coord2), 1.0, Scalar( 255, 0, 0 ) );
	circle( canvas, Point( pointer, coord3), 1.0, Scalar( 255, 0, 255 ) );
	// Use currently increment pixel points by increment frame.
	// where we assume frames move with constant interval.

// Shift image to the left.
	imshow("Canvas", canvas);
	waitKey(1);
}
int dSize = 960;
Mat dashBoard = cvCreateImage(cvSize(dSize, dSize), IPL_DEPTH_32F, 3);

void initialiseDashBoard() {
//	rectangle(dashBoard, cvRect(0,0, 480, 480), Scalar(255, 255, 255), CV_FILLED );
	circle(dashBoard, Point(dSize/2,dSize/2), 400, Scalar(255, 255, 255),CV_FILLED);
	int step = 360/36;
	for (int i = -180; i <= 180; i += step) {
		line(dashBoard, Point((dSize/2) + (2*i), (dSize/2) -50), Point((dSize/2) + (2*i), (dSize/2) + 50), Scalar(0, 0, 0)); // vertical line.
		char text[10];
		sprintf(text, "%d", i);
		putText(dashBoard,(const char*)text, Point((dSize/2) + (2*i) + 5 ,dSize/2 + 5),FONT_HERSHEY_PLAIN, 0.5,Scalar(0,0,255) );
		line(dashBoard, Point(dSize/2 - 50,(dSize/2) + (2*i)), Point( dSize/2 + 50,(dSize/2) + (2*i)), Scalar(0, 0, 0)); // horizontal line.
		putText(dashBoard,(const char*)text, Point(dSize/2,(dSize/2) - (2*i) + 5),FONT_HERSHEY_PLAIN, 0.5,Scalar(0,0,255) );
	}
}

float radsToDegrees(float angle) {
	angle *= 180/CV_PI;
		if (angle > 180) {
			angle -= 360;
		} else if (angle < -180) {
			angle += 360;
		}
		return angle;
}

void drawOrientation(Mat pose) {
	initialiseDashBoard();
	// orientation is usually in radians. Need to transform to degrees.
	float roll = -2*radsToDegrees(pose.at<float>(0,3));
	float yaw =  2*radsToDegrees(pose.at<float>(0,4));
	float pitch = pose.at<float>(0,5);

	Mat centrePoint;
	centrePoint.create(2,1, CV_32F);

	// Account for roll
	centrePoint.at<float>(0,1) = dSize/2 + roll; // y-axis
	centrePoint.at<float>(0,0) = dSize/2; // x-axis

	// Account for yaw
	centrePoint.at<float>(0,0) += yaw; // x-axis

	// Account for pitch
	Mat T = (Mat_ <float>(2,2) << cos(pitch), -sin(pitch), sin(pitch), cos(pitch) );
	Mat pointl = (Mat_<float>(2,1) << -50, 0);
	Mat pointr = (Mat_<float>(2,1) << 50, 0);
	pointl = T*pointl;
	pointr = T*pointr;

	// find middle point
	float x = centrePoint.at<float>(0,0);
	float y = centrePoint.at<float>(0,1);

	// draw circle at middle
	circle(dashBoard, Point(x,y), 3, Scalar(0,255, 0));
	cout << "Pointl" << pointl << endl << endl;
	// Draw orientation line
	line(dashBoard, Point(pointl.at<float>(0,0) + centrePoint.at<float>(0,0), -pointl.at<float>(0,1)+centrePoint.at<float>(0,1)),Point(pointr.at<float>(0,0) + centrePoint.at<float>(0,0), -pointr.at<float>(0,1)+centrePoint.at<float>(0,1)),Scalar(0,255, 0)  );
	imshow("DashBoard", dashBoard);
	waitKey(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam1");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
//  const ros::Duration timeout(0, 100000000); // 100 milli second.
  int noCams = 2;

  n.getParam("noCams", noCams);

  Mat T;
  T.create(4, 4, CV_32FC1);
  initialiseTrans();
  initialiseDashBoard();
  // Initialise all global variables.
  struct Config
  {
    CaptureInfo *captureInfo;
    CameraInfo *cameraInfo;
    Marker marker;
//    TrackerMultiMarker * tracker;
  };

  Config config;
  config.captureInfo = new CaptureInfo[noCams];
  config.cameraInfo = new CameraInfo[noCams];
  config.marker.marker_pose = Mat::zeros(6, 1, CV_32F);

  for (int i = 0; i < noCams; i++)
  {
    config.cameraInfo[i].camPose = Mat::zeros(6, 1, CV_32F);
    config.cameraInfo[i].T = Mat::zeros(4, 4, CV_32F);
    config.cameraInfo[i].cameraPoseKnown = false;
    initialiseCam(i, config.captureInfo[i].capture);
    Mat frame;
    //Todo: May need to make width and height global like tracker
    config.captureInfo[i].capture.read(frame);
    initFrameSize(config.captureInfo[i].capture, config.captureInfo[i].width, config.captureInfo[i].height);
//    configTracker(config.captureInfo[i].tracker, config.captureInfo[i].width, config.captureInfo[i].height);
    config.captureInfo[i].newResult = false;
  }
  configTracker(config.captureInfo[0].width, config.captureInfo[0].height);

// Initialise known camera's position.
  stringstream stream;
  string pose_s;
  int x, y, z, roll, yaw, pitch;
  n.getParam("pose", pose_s);
  stream << pose_s;
  stream >> x >> y >> z >> roll >> yaw >> pitch;
  config.cameraInfo[0].camPose = (Mat_<float>(6, 1) << x, y, z, roll, yaw, pitch);
  // Todo:This is for debug only.
  config.cameraInfo[0].camPose = Mat::zeros(6, 1, CV_32F);
  config.cameraInfo[0].cameraPoseKnown = true;
  writeCalibrationToFile(0, config.cameraInfo[0].camPose);

  IplImage* imgArr[noCams];
  int dataSize = 500;
  float stats[6][dataSize];
  for (int id = 0; id < noCams; id++)
  {
    Mat temp;
    config.captureInfo[id].capture.read(temp);
    imgArr[id] = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U, temp.channels());
  }

  while (ros::ok())
  {

    for (int id = 0; id < noCams; id++)
    {

      Mat temp;
      config.captureInfo[id].capture.read(temp);
//      int frameTime = config.captureInfo[id].capture.get(CV_CAP_PROP_POS_MSEC);
      int frameTime = -1;
//      cout << "Frame time: " << frameTime << " msecs; id: "<<id<< endl;
      if (temp.empty())
      {
        ROS_INFO("ERROR: files to grab new image\n");
        exit(EXIT_FAILURE);
      }

      imgArr[id]->imageData = (char*)temp.data;

      config.captureInfo[id].newResult = processMarkerImg(imgArr[id], tracker, config.captureInfo[id].width,
                                                          config.captureInfo[id].height,id); // Get transformation matrix from new result.

      if (config.cameraInfo[id].cameraPoseKnown)
      {
        // If camera's position is known then start publishing marker position
        // publish marker pose.
//        cout << "cam_pose" << endl << config.cameraInfo[id].camPose << endl << endl;
        if (config.captureInfo[id].newResult)
        {
          config.marker.mTime = frameTime;
          updateMarkerPose(tracker, config.cameraInfo[id].camPose, config.marker,config.cameraInfo[id].T);
//          drawOrientation(config.marker.marker_pose);
//          drawTrans(config.marker.marker_pose);
//          cout << "Marker Pose from id: " << id << endl << config.marker.marker_pose << endl << endl;
          config.captureInfo[id].newResult = false;
        }
      }
      else
      { // Derive Camera's position from marker image.
//        cout << "Unknown Cam Pose" << endl;

        if (config.captureInfo[id].newResult)
//          if (config.captureInfo[id].newResult && ((frameTime - config.marker.mTime) < timeout))
        {
//          cout << "Getting cam pose" << endl;
          getCameraPose(tracker, &config.marker.marker_pose, &config.cameraInfo[id].camPose,config.cameraInfo[id].T);
//          config.cameraInfo[id].cameraPoseKnown = true;
          config.captureInfo[id].newResult = false;
//          cout << "cam_pose: " << config.cameraInfo[id].camPose.at<float>(0,0) << "; marker: " << config.marker.marker_pose.at<float>(0,0) <<"id :" << id << endl << endl;
//          drawTrans(config.cameraInfo[id].camPose);
          //          cout << "; cam0-T:" << config.cameraInfo[0].T<<endl;// << "; cam1-T:" << config.cameraInfo[1].T.at<float>(3,0) << "; diff:"
//        		 << config.cameraInfo[0].T.at<float>(3,0) - config.cameraInfo[1].T.at<float>(3,0) << endl << endl;
          drawOrientation(config.cameraInfo[id].camPose);
          cout << "cam_pose,  id: "<< id << endl << config.cameraInfo[id].camPose << endl << endl;
          static int count;
          if (count < dataSize) {
			  stats[0][count] =config.cameraInfo[id].camPose.at<float>(0,0);
			  stats[1][count] =config.cameraInfo[id].camPose.at<float>(0,1);
			  stats[2][count] =config.cameraInfo[id].camPose.at<float>(0,2);
			  stats[3][count] =config.cameraInfo[id].camPose.at<float>(0,3);
			  stats[4][count] =config.cameraInfo[id].camPose.at<float>(0,4);
			  stats[5][count] =config.cameraInfo[id].camPose.at<float>(0,5);
			  count ++;
			  cout << "Count: " << count << endl << endl;
          }else if (count == dataSize){
        	  // Calculate std and mean.
        	  float mean[6] = {0,0,0,0,0,0};
        	  float stdev[6] = {0,0,0,0,0,0};
        	  for (int i = 0; i < 1000; i++) {
        		  mean[0] += stats[0][i];
        		  mean[1] += stats[1][i];
        		  mean[2] += stats[2][i];
        		  mean[3] += stats[3][i];
        		  mean[4] += stats[4][i];
        		  mean[5] += stats[5][i];
        	  }
        	  mean[0] /= dataSize;
        	  mean[1] /= dataSize;
        	  mean[2] /= dataSize;
        	  mean[3] /= dataSize;
        	  mean[4] /= dataSize;
        	  mean[5] /= dataSize;
        	  // get stdev
        	  for (int i = 0; i < dataSize; i ++) {
        		  stdev[0] =pow(( mean[0]-stats[0][i]),2);
        		  stdev[1] =pow(( mean[1]-stats[1][i]),2);
        		  stdev[2] =pow(( mean[2]-stats[2][i]),2);
        		  stdev[3] =pow(( mean[3]-stats[3][i]),2);
        		  stdev[4] =pow(( mean[4]-stats[4][i]),2);
        		  stdev[5] =pow(( mean[5]-stats[5][i]),2);
        	  }
        	  stdev[0]= sqrt(stdev[0]/=dataSize);
        	  stdev[1]= sqrt(stdev[1]/=dataSize);
        	  stdev[2]= sqrt(stdev[2]/=dataSize);
        	  stdev[3]= sqrt(stdev[3]/=dataSize);
        	  stdev[4]= sqrt(stdev[4]/=dataSize);
        	  stdev[5]= sqrt(stdev[5]/=dataSize);
        	  cout << "mean" << endl<< "[" << mean[0]<<";"<<endl<< mean[1]<<";"<<endl<< mean[2]<<";"<<endl<< mean[3]<<";"<<endl<< mean[4]<<";"<<endl<< mean[5]<<"]"<<endl<<endl;
        	  cout << "stdev" << endl<< "["<< stdev[0]<<";"<<endl<< stdev[1]<<";"<<endl<< stdev[2]<<";"<<endl<< stdev[3]<<";"<<endl<< stdev[4]<<";"<<endl<< stdev[5]<<";"<<endl;
        	  count ++;
        	  config.cameraInfo[id].cameraPoseKnown = true;
        	  //Update camepose
        	  config.cameraInfo[id].camPose.at<float>(0,0) = mean[0];
        	  config.cameraInfo[id].camPose.at<float>(0,1) = mean[1];
        	  config.cameraInfo[id].camPose.at<float>(0,2) = mean[2];
        	  config.cameraInfo[id].camPose.at<float>(0,3) = mean[3];
        	  config.cameraInfo[id].camPose.at<float>(0,4) = mean[4];
        	  config.cameraInfo[id].camPose.at<float>(0,5) = mean[5];
        	  writeCalibrationToFile(id, config.cameraInfo[id].camPose);
          }
        }
      }

      //check if  if all cameras' poses are known
      bool shut = true;
      for (int i = 0; i < noCams; i++) {
    	  if (config.cameraInfo[i].cameraPoseKnown == false) {
    		  shut = false;
    	  }
      }
      if (shut == true){
    	  cout << "Shutting down!"<< endl;
    	  ros::shutdown();
    	  return 0;
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
