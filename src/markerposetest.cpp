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

float pastGain = 1.5;
float pitchGain = 1.5;
float pastAveErr = 99999;
float errRootSum = 0;
float counter = 1;
float sampleSize = 20;
bool updatePitchGain(float &pastGain, float &pastAveErr, float &counter, float &pitchGain, Mat & actualMarkerPose, Mat & estimatedMarkerPose){

	if (counter == sampleSize) {
		float currAveErr = errRootSum/counter;
		errRootSum = norm(actualMarkerPose-estimatedMarkerPose);
		counter = 1;

		if (pastAveErr > (currAveErr)) {
			pastGain = pitchGain;
			pastAveErr = currAveErr;
			pitchGain *= 1.06;
		} else {
			pitchGain = pastGain;
			return false;
		}
	} else{
		errRootSum += norm(actualMarkerPose-estimatedMarkerPose);
		counter ++;
	}
	return true;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "cam1");

	ros::NodeHandle n("~");
	ros::Rate loop_rate(10);
	int noCams = 2;

	n.getParam("noCams", noCams);

	Mat T;
	T.create(4, 4, CV_32FC1);
	// Initialise all global variables.

	bool pitchStable = false;
	float timeout = 100; //ms
	TrackerHelper::Config config;
	config.captureInfo = new TrackerHelper::CaptureInfo[noCams];
	config.cameraInfo = new TrackerHelper::CameraInfo[noCams];
	config.marker.marker_pose = Mat::zeros(6, 1, CV_32F);
	Mat markerPose_t = Mat::zeros(6,1, CV_32FC1);
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
	helper.configTracker( config.captureInfo[0].width,
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
			config.captureInfo[id].capture.read(temp);
			config.captureInfo[id].capture.read(temp);
			config.captureInfo[id].capture.read(temp);
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
					helper.calcMarkerPose(helper.tracker, config.cameraInfo[id].camPose, config.marker.marker_pose,
															config.marker.Tm);
					cout << "Marker Pose:"<< endl << config.marker.marker_pose << endl;
				}
			} else if ((numDetected > 0)
					& ((frameTime - config.marker.mTime) < timeout)) {
				cout << "cam,  id: " << id<< endl;
				helper.getCameraPose(helper.tracker, &config.marker.marker_pose,&config.cameraInfo[id].camPose,
						config.marker.Tm, pitchGain);
				cout << "Camera Pose" << endl<<config.cameraInfo[id].camPose << endl;
				helper.calcMarkerPose(helper.tracker, config.cameraInfo[id].camPose, markerPose_t,
										config.marker.Tm);
				cout << "Marker Pose:"<< endl << config.marker.marker_pose << endl;
				cout << "Marker Pose_t:"<< endl << markerPose_t << endl;
				if (!pitchStable){
				bool result = updatePitchGain(pastGain, pastAveErr,counter,pitchGain, config.marker.marker_pose, markerPose_t);
				if (!result) {
					pitchStable = true;
				}
				}
//				 put data in file.
				bool invalidMarkerPose = false;
								for (int i = 0; i < 6; i++) {
									float test = config.cameraInfo[id].camPose.at<float>(i, 0);
									if (test != test) {
										invalidMarkerPose = true;
									} else if (abs(test) > 1e4) {
										invalidMarkerPose = true;
									}
								}
								if ((!invalidMarkerPose)& pitchStable){
				for (int i = 0; i < 6 ; i++) {
						fp << config.cameraInfo[id].camPose.at<float>(i,0);
						if (i==5){
							 fp<<"\n";
						}else{
							fp <<",";
						}
				}
				count++;
				cout << "Count: " << count << endl;
								}
			}

			ros::spinOnce();

			loop_rate.sleep();
		}
		if (count == 200) {
			break;
			fp.close();
			config.captureInfo[0].capture.release();
			config.captureInfo[1].capture.release();
		}
	}
}
