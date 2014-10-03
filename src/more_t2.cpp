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

// Standard C++ entry point

int initialiseTracker() {
	tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
	//tracker.setLoadUndistLUT(true);

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

	// define size of the marker in OpenGL units
//	tracker.setPatternWidth(2.0);

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

int main(int argc, char** argv) {

	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);
	printf("EST!!!");
	if (initialiseTracker() != 0)
		return 0;

	const char * filename = "/home/parallels/catkin_ws/src/more_t2/data/Untitled.jpg";
//	const char* filename ="/home/parallels/catkin_ws/src/more_t2/videos/test.mov";
	/*IplImage* frame = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);*/
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	//IplImage *tempImg = cvCreateImage( cvSize(width, height), IPL_DEPTH_8U, 1 );
	IplImage *tempImg2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	IplImage* img;
	CvCapture* capture = cvCaptureFromCAM(1);



	const int bpp = 1;
	size_t numPixels = width * height * bpp;
	    size_t numBytesRead;
	    const char *fName = "/home/parallels/local/tools/ARToolKitPlus-2.3.0/sample/data/markerboard_480-499.raw";
	    unsigned char cameraBuffer[numPixels];

	    // try to load a test camera image.
	    // these images files are expected to be simple 8-bit raw pixel
	    // data without any header. the images are expetected to have a
	    // size of 320x240.
	    if (FILE* fp = fopen(fName, "rb")) {
	        numBytesRead = fread(cameraBuffer, 1, numPixels, fp);
	        fclose(fp);
	    } else {
	        printf("Failed to open %s\n", fName);
	        return -1;
	    }

	    if (numBytesRead != numPixels) {
	        printf("Failed to read %s\n", fName);
	        return -1;
	    }


	if (!capture) {
		std::cout << "Could not initialize capturing...\n" << std::endl;
		return -1;
	}
	while (ros::ok()) {
//		img = cvQueryFrame(capture);
		img = cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
		if (!img) {
			printf("Failed to open file");
			break;
		}
//		cvNamedWindow("image", CV_WINDOW_AUTOSIZE);

		// corp the image
//		cvSetImageROI(img, cvRect(0, 0, 640, 480));
//
//		IplImage *tmp = cvCreateImage(cvGetSize(img), img->depth,
//				img->nChannels);
//
//		cvCopy(img, tmp, NULL);
//
//		IplImage *tempImg = cvCreateImage(cvGetSize(tmp), tmp->depth, 1);
//		cvCvtColor(tmp, tempImg, CV_RGB2GRAY);
//		cvResize(tempImg, tempImg2);
//		cvThreshold(tempImg2, greyImg, (float) 70, 255.0, CV_THRESH_BINARY);
//		cvShowImage("MyVideo", tempImg2);
//		cvWaitKey(10);
		int numDetected = tracker.calc((unsigned char*) img->imageData);
//		int numDetected = tracker.calc(cameraBuffer);
		if (numDetected != 0) {
			printf("Yes!");
			return 0;
		}

		// use the result of calc() to setup the OpenGL transformation
		// glMatrixMode(GL_MODELVIEW)
		// glLoadMatrixf(tracker.getModelViewMatrix());

		ROS_INFO("\n\nNum of markers: %d  \n\nPose-Matrix:\n  ", numDetected);
		for (int i = 0; i < 16; i++)
			printf("%.2f  %s", tracker.getModelViewMatrix()[i],
					(i % 4 == 3) ? "\n  " : "");
		ros::spinOnce();

		loop_rate.sleep();

	}
	cvReleaseCapture(&capture);
	return 0;
}
