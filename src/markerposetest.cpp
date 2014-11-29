/*
 * camera_pose_calibration.cpp
 *
 *  Created on: 15 Oct 2014
 *      Author: parallels
 */

#include "trackerhelper.h"
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

TrackerHelper helper;
struct myPoint {
	float x;
	float y;
	float z;
};
Mat bin[4];
float data[12];

void insertToBin(Mat& pose) {
	float dist = 99999;
	int index = 0;
	for (int i = 0; i < 4; i++) {
		float test = norm(pose.rowRange(0, 3) - bin[i]);
		if (test < dist) {
			dist = test;
			index = i;
		}
	}
	data[index * 3] = pose.at<float>(0, 0);
	data[index * 3 + 1] = pose.at<float>(1, 0);
	data[index * 3 + 2] = pose.at<float>(2, 0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cam");

	ros::NodeHandle n("~");
	ros::Rate loop_rate(10);

	Mat T;
	T.create(4, 4, CV_32FC1);
	TrackerHelper::TransGraph transGraph;
	TrackerHelper::RotGraph rotGraph;

	/********************Initialise all global variables.********************/
	int noCams = 1;
	int id = 0;
	TrackerHelper::Config config;
	config.captureInfo = new TrackerHelper::CaptureInfo[noCams];
	config.cameraInfo = new TrackerHelper::CameraInfo[noCams];
	config.marker.marker_pose = Mat::zeros(6, 1, CV_32F);

	config.cameraInfo[0].camPose = Mat::zeros(6, 1, CV_32F);
	config.cameraInfo[0].T = Mat::zeros(4, 4, CV_32F);
	config.cameraInfo[0].cameraPoseKnown = false;
	helper.initialiseCapture(id, config.captureInfo[0].capture);
	Mat frame;
	//Todo: May need to make width and height global like tracker
	config.captureInfo[0].capture.read(frame);
	helper.initFrameSize(config.captureInfo[0].capture,
			config.captureInfo[0].width, config.captureInfo[0].height);
//    configTracker(config.captureInfo[i].tracker, config.captureInfo[i].width, config.captureInfo[i].height);
	config.captureInfo[0].newResult = false;

	helper.configTracker(config.captureInfo[0].width,
			config.captureInfo[0].height);

// Initialise known camera's position.
	// Todo:This is for debug only.
	config.cameraInfo[0].camPose = Mat::zeros(6, 1, CV_32F);
	config.cameraInfo[0].cameraPoseKnown = true;

	Mat temp;
	config.captureInfo[0].capture.read(temp);
	IplImage *img = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U,
			temp.channels());

	stringstream sbuffer;
	string path = "/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_";
	sbuffer << path << id << ".csv";
	ofstream fp;
	fp.open(sbuffer.str().c_str(), ios::out | ios::trunc);
	sbuffer.str("");
	if (!fp.is_open()) {
		cout << "Error! Cannot open file to save pose data" << endl;
		exit(EXIT_FAILURE);
	}

	int count = 0;

	bool first = true;
	while (ros::ok()) {

		Mat temp;
		config.captureInfo[0].capture.read(temp);
		if (temp.empty()) {
			ROS_INFO("ERROR: files to grab new image\n");
			exit(EXIT_FAILURE);
		}

		img->imageData = (char*) temp.data;

		int numDetected = helper.getNumDetected(img, helper.tracker,
				config.captureInfo[0].width, config.captureInfo[0].height, 0); // Get transformation matrix from new result.

		if (numDetected == 4) {
			for (int i = 0; i < numDetected; i++) {
				helper.getMarkerPose(helper.tracker, i,
						config.marker.marker_pose);
				if (first) {
					// Allocate the initial locations in data.
					data[3 * i] = config.marker.marker_pose.at<float>(0, 0);
					data[3 * i + 1] = config.marker.marker_pose.at<float>(1, 0);
					data[3 * i + 2] = config.marker.marker_pose.at<float>(2, 0);
					bin[i] = Mat::zeros(3, 1, CV_32FC1);
					config.marker.marker_pose.rowRange(0, 3).copyTo(bin[i]);
				} else {
					// Cluster recent data into bin.
					insertToBin(config.marker.marker_pose);
				}
			}
			first = false;

			fp << data[0] << "," << data[1] << "," << data[2] << "," << data[3]
					<< "," << data[4] << "," << data[5] << "," << data[6] << ","
					<< data[7] << "," << data[8] << "," << data[9] << ","
					<< data[10] << "," << data[11] << "\n";

			count++;
			cout << "Count: " << count << endl;
		}

		if (count == 1500) {
			fp.close();
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	config.captureInfo[id].capture.release();

}
