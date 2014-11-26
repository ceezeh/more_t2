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

int main(int argc, char** argv) {
	ros::init(argc, argv, "cam1");

	ros::NodeHandle n("~");
	ros::Rate loop_rate(10);
	int noCams = 2;

	n.getParam("noCams", noCams);

	Mat T;
	T.create(4, 4, CV_32FC1);
	// Initialise all global variables.

	float timeout = 100; //ms
	TrackerHelper::Config config;
	config.captureInfo = new TrackerHelper::CaptureInfo[noCams];
	config.cameraInfo = new TrackerHelper::CameraInfo[noCams];
	config.marker.marker_pose = Mat::zeros(6, 1, CV_32F);
	config.marker.Tm = Mat::eye(4, 4, CV_32F);
	config.marker.mTime = -99999;
	for (int i = 0; i < noCams; i++) {
		config.cameraInfo[i].camPose = Mat::zeros(6, 1, CV_32F);
		config.cameraInfo[i].Tc = Mat::eye(4, 4, CV_32F);
		config.cameraInfo[i].cameraPoseKnown = false;
		helper.initialiseCapture(i, config.captureInfo[i].capture);
		Mat frame;
		config.captureInfo[i].capture.read(frame);
		helper.initFrameSize(config.captureInfo[i].capture,
				config.captureInfo[i].width, config.captureInfo[i].height);
		config.captureInfo[i].newResult = false;
	}
	helper.configTracker(config.captureInfo[0].width,
			config.captureInfo[0].height);

	config.cameraInfo[0].cameraPoseKnown = true;

	IplImage* imgArr[noCams];
	for (int id = 0; id < noCams; id++) {
		Mat temp;
		config.captureInfo[id].capture.read(temp);
		imgArr[id] = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U,
				temp.channels());
	}

	string path = "/home/parallels/catkin_ws/src/more_t2/posedata/cam.csv";
	ofstream fp;
	fp.open(path.c_str(), ios::out | ios::trunc);
	if (!fp.is_open()) {
		cout << "Error! Cannot open file to save pose data" << endl;
		exit(EXIT_FAILURE);
	}
	int count = 0;
	while (ros::ok()) {

		for (int id = 0; id < noCams; id++) {

			Mat temp;
			config.captureInfo[id].capture.read(temp);
			int frameTime = config.captureInfo[id].capture.get(
					CV_CAP_PROP_POS_MSEC);
			if (temp.empty()) {
				ROS_INFO("ERROR: files to grab new image\n");
				exit(EXIT_FAILURE);
			}

			imgArr[id]->imageData = (char*) temp.data;

			int numDetected = helper.getNumDetected(imgArr[id], helper.tracker,
					config.captureInfo[id].width, config.captureInfo[id].height,
					id); // Get transformation matrix from new result.

			if (config.cameraInfo[id].cameraPoseKnown) {
				if (numDetected > 0) {
					config.marker.mTime = frameTime;
					cout << "Cam id: " << id << endl;
					helper.getMarkerT(helper.tracker, 0, config.marker.Tm,config.cameraInfo[id].Tc);
					cout << "getMarker Tm:" << config.marker.Tm << endl <<"Tc" << config.cameraInfo[id].Tc << endl;
				}
			} else if ((numDetected > 0)
					& ((frameTime - config.marker.mTime) < timeout)) {
				cout << "cam,  id: " << id<< endl;
				cout << "getMarker Tm:" << config.marker.Tm << endl;
				helper.getCameraT(helper.tracker, 0, config.cameraInfo[id].Tc,
						config.marker.Tm);
				cout << "getMarker Tm:" << config.marker.Tm << endl <<"Tc" << config.cameraInfo[id].Tc << endl;
				helper.getMarkerT(helper.tracker, 0, config.marker.Tm,config.cameraInfo[id].Tc);
				cout << "getMarker Tm:" << config.marker.Tm << endl <<"Tc" << config.cameraInfo[id].Tc << endl;
				// put data in file.
				fp << config.cameraInfo[id].Tc.at<float>(0, 3) << ","
						<< config.cameraInfo[id].Tc.at<float>(1, 3) << ","
						<< config.cameraInfo[id].Tc.at<float>(2, 3) << "\n";
				count++;
				cout << "Count: " << count << endl;
			}

			ros::spinOnce();

			loop_rate.sleep();
		}
		if (count == 1500) {
			break;
			fp.close();
			config.captureInfo[0].capture.release();
			config.captureInfo[1].capture.release();
		}
	}
}
