// Include the ROS C++ APIs
#include <ros/ros.h>
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <cv.h>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

const int width = 320;
const int height = 240;
TrackerMultiMarker tracker(width, height, 8, 6, 6, 6, 0);
Mat centreMat = (Mat_<float>(4,1) << 0, 0, 0, 1);
Mat offset  = (Mat_<float>(4,1) << -width/2,height/2, 0, 1);
// Standard C++ entry point

int configTracker() {
	tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

	// load a camera file.
	if (!tracker.init(
			"/home/parallels/catkin_ws/src/more_t2/data/no_distortion.cal",
			"/home/parallels/local/tools/ARToolKitPlus-2.3.0/sample/data/markerboard_480-499.cfg",
			1.0f, 1000.0f)) // load MATLAB file
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

void calcPoseMatrix(TrackerMultiMarker* t, Mat* pose) {
	const ARFloat* tf = t->getModelViewMatrix();
	Mat A = Mat(4, 4, CV_32FC1, (float *)tf);
//	cout <<"[Debug] A= " << A.t() <<endl;
	*pose  = A.t() * centreMat;
	cout << "pose = "<< endl << *pose << endl << endl;
}
// Write pose to file.

// This functions draws a real-time 2D graph.
int cwidth  = 6*width;
Mat canvas = cvCreateImage(cvSize(cwidth, height), IPL_DEPTH_8U, 1);

void initialiseCanvas() {
	rectangle(canvas, cvRect(0,0, cwidth, height), 255, CV_FILLED );
	line( canvas, Point( 0, height/2 ), Point( cwidth, height/2), 0 );
}
void drawGraphs(Mat pose) {
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
		initialiseCanvas();
	}
	float ycoord = height/2 - pose.at<float>(0,0)*height/400; // For x represent 40 cm in a whole.
	cout << "ycoord"<< ycoord << endl << endl;
	circle( canvas, Point( pointer, ycoord), 1.0, Scalar( 0, 255, 0 ) );
	// Use currently increment pixel points by increment frame.
	// where we assume frames move with constant interval.
	// Shift image to the left.
	imshow("Canvas", canvas);
	waitKey(1);
}
int main(int argc, char** argv) {

	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	if (configTracker() != 0)
		return 0;
	initialiseCanvas();
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	IplImage *tempImg2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	IplImage* img;
	CvCapture* capture = cvCaptureFromCAM(1);
	Mat pose;
	if (!capture) {
		std::cout << "Could not initialize capturing...\n" << std::endl;
		return -1;
	}
	while (ros::ok()) {
		img = cvQueryFrame(capture);
		if (!img) {
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
		cvShowImage("MyVideo", greyImg);
		cvWaitKey(10); // Wait for image to be rendered on screen. If not included, no image is shown.
		int numDetected = tracker.calc((unsigned char*) greyImg->imageData);
		if (numDetected != 0) {
			printf("Num = %d\n\n", numDetected);
			calcPoseMatrix(&tracker, &pose);
			drawGraphs(pose);
		}

		ros::spinOnce();

		loop_rate.sleep();

	}
	cvReleaseCapture(&capture);
	return 0;
}
