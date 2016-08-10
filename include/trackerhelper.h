/*
 * more_t2.h
 *
 *  Created on: Nov 18, 2014
 *      Author: parallels
 */

#ifndef MORE_T2_H_
#define MORE_T2_H_
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>      // std::ofstream
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>
#include <math.h>
#include <cstdlib>
 #include <unistd.h>
#include <new>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;
typedef unsigned long long timestamp_t;

class TrackerHelper {

public:
	TrackerMultiMarker *tracker;
	TrackerHelper() {
		centreMat = (Mat_<float>(4, 1) << 0, 0, 0, 1);
		tracker = NULL;
		markerWidth = 0;
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
		Mat Tc;
		bool cameraPoseKnown;
	};
	struct Marker {
		Mat marker_pose;
		Mat Tm;
		int mTime;
	};
	struct Config {
		CaptureInfo *captureInfo;
		CameraInfo *cameraInfo;
		Marker marker;
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


	class ImageReader{
		int index;
		stringstream dirname;
		VideoCapture cap;
		ifstream timefile;
	public:
		ImageReader(string dirname) {
			index = 1;
			stringstream temp;
			temp << dirname << "/vid.mov";
			cap.open(temp.str().c_str());
			if(!cap.isOpened()) {
				cout << "sorry. video could not be read" << endl;
				exit(1);
			}
			temp.str("");
			temp << dirname << "/timestamps.txt";
			timefile.open(temp.str().c_str(), ios::in);
			if(timefile.bad()) {
				cout << "sorry. time file could not be read" << endl;
				exit(1);
			}
		}
	~ImageReader() {
		timefile.close();
		cap.release();
	}
		void read(Mat& temp, float &timestamp);
		void vidRead(Mat& temp, timestamp_t &timestamp);
		void read(Mat& temp);
	};
	void setMarkerWidth(int width) {
		markerWidth = width;
	}
	void initialiseCapture(int id, VideoCapture &cap);
	void initFrameSize(Mat frame, int &width, int &height);
	int configTracker(int width, int height, const char* cal,const char* cfg);

	void calRotYXZ(float pitch, float yaw, float roll, float Ys, float Zs, Mat &R);
	void getAnglesYXZ(float &pitch, float &yaw, float &roll,  float &Ys, float &Zs, Mat R);

	void get6DOFMarkerPose(TrackerMultiMarker* tracker,	 Mat cam_pose,	Mat &marker_pose, int index);
	void calcMarkerPose(TrackerMultiMarker* tracker, Mat &marker_pose, Mat &T);
	bool processMarkerImg(IplImage *img, TrackerMultiMarker *tracker, int width,
			int height, int id);
	int getNumDetected(IplImage *img, TrackerMultiMarker *tracker,
		int width, int height, char * name);
	void getMarkerPosition(TrackerMultiMarker* tracker, int index, Mat &pose);
	bool calcBigMarkerPose(TrackerMultiMarker* tracker,		Mat &marker_pose, Mat &T, bool firstCam);

	void getCameraPose(Mat* from_pose, Mat* to_pose, Mat* to_campose);
	void getCameraPose(TrackerMultiMarker* tracker, Mat* cam_pose);
	void saveCamPose(int id, Mat cam_pose);

	void initialiseTrans();
	void drawTrans(Mat trans, const char* title);
	float radsToDegrees(float angle);
	void drawOrientation(Mat pose, const char* title);
	void getCamPose(Mat * poseArray, int index);
	void clusterData(Mat samples, Mat &centers);
private:
	void initialiseDashBoard();
	Mat centreMat;
	int markerWidth;
};

#endif /* MORE_T2_H_ */
