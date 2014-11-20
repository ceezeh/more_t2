/*
 * more_t2.h
 *
 *  Created on: Nov 18, 2014
 *      Author: parallels
 */

#ifndef MORE_T2_H_
#define MORE_T2_H_
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

class TrackerHelper {

public:
	TrackerMultiMarker *tracker;
	TrackerHelper() {
		centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);
		tracker = NULL;
	}
	struct CaptureInfo {
		int width;
		int height;
		VideoCapture capture;
		bool newResult;
		//  TrackerMultiMarker *tracker;
	};
	struct CameraInfo {
		Mat camPose;
		Mat T;
		bool cameraPoseKnown;
	};
	struct Marker {
		Mat marker_pose;
		int mTime;
	};
	struct Config {
		CaptureInfo *captureInfo;
		CameraInfo *cameraInfo;
		Marker marker;
		//    TrackerMultiMarker * tracker;
	};
	class TransGraph{
	public:
		int width;
		int height;
		Mat xcanvas;
		Mat ycanvas;
		Mat zcanvas;
		int pointer;
		TransGraph(){
			pointer = 0;
			width = 4*320;
			height = 800;
			xcanvas = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
			ycanvas = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
			zcanvas = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
			initialiseTrans();
		}
		void initialiseTrans();
		void drawTrans(Mat trans, const char* title);
	};

	class RotGraph{
	public:
		int dSize;
		Mat dashBoard;
		RotGraph(){
			dSize = 960;
			dashBoard = cvCreateImage(cvSize(dSize, dSize), IPL_DEPTH_32F, 3);
			initialiseDashBoard();
		}
		float radsToDegrees(float angle);
		void initialiseDashBoard();
		void drawOrientation(Mat pose, const char* title);
	};

	void initialiseCapture(int id, VideoCapture &cap);
	void initFrameSize(VideoCapture &cap, int &width, int &height);
	int configTracker(int width, int height);
	void getRotMatrix(float roll, float yaw, float pitch, Mat &R);
	void calcMarkerPose(TrackerMultiMarker* tracker, Mat cam_pose,
			Mat &marker_pose, Mat &T);
	bool processMarkerImg(IplImage *img, TrackerMultiMarker *tracker, int width,
			int height, int id);
	void getCameraPose(TrackerMultiMarker* tracker, Mat* marker_pose_t,
			Mat* cam_pose, Mat&T);
	void writeCalibrationToFile(int id, Mat cam_pose);
	void initialiseTrans();
	void drawTrans(Mat trans, const char* title);

	float radsToDegrees(float angle);
	void drawOrientation(Mat pose, const char* title);
	void getCamPose(Mat * poseArray, int index);
	void clusterData(Mat samples, Mat &centers);
private:
	void initialiseDashBoard();
	Mat centreMat;
};

#endif /* MORE_T2_H_ */
