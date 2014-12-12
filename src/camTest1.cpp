/*
 * camTest1.cpp
 *
 *  Created on: Nov 29, 2014
 *      Author: parallels
 */

#include "trackerhelper.h"
#include <geometry_msgs/Pose.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

TrackerHelper helper;

int id = 1;
ros::Publisher markerPose_pub;
// The pose of the initial camera.
Mat marker_pose;
TrackerHelper::Config config;
bool newMarker = false;
int frameTime = -9999;
int newMarkerTime = -9999;
int timeout = 100;
ros::Subscriber markerPose_sub;

// Updates and publishes marker pose.

float pitchGain = 0.9;
float pastGain = pitchGain;
float pastAveErr = 99999;
float errRootSum = 0;
float counter = 1;
float sampleSize = 40;
bool updatePitchGain(float &pastGain, float &pastAveErr, float &counter,
		float &pitchGain, Mat & actualMarkerPose, Mat & estimatedMarkerPose) {

	if (counter == sampleSize) {
		float currAveErr = errRootSum / counter;
		errRootSum = norm(actualMarkerPose - estimatedMarkerPose);
		counter = 1;

		if (pastAveErr > (currAveErr)) {
			pastGain = pitchGain;
			pastAveErr = currAveErr;
			pitchGain += abs(pitchGain)*.1;
		} else {
			pitchGain = pastGain;
			return false;
		}
	} else {
		errRootSum += norm(actualMarkerPose - estimatedMarkerPose);
		counter++;
	}
	return true;
}
bool isInvalidPose(Mat &pose) {
	for (int i = 0; i < 6; i++) {
		float test = pose.at<float>(i, 0);
		if (test != test) {
			return true;
		} else if (abs(test) > 1e9) {
			return true;
		}
	}
	return false;

}
void markerPoseCallback(const geometry_msgs::Pose & msg) {
	if ((frameTime - msg.orientation.w) < timeout) {
		marker_pose.at<float>(0, 0) = msg.position.x;
		marker_pose.at<float>(0, 1) = msg.position.y;
		marker_pose.at<float>(0, 2) = msg.position.z;
		marker_pose.at<float>(0, 3) = msg.orientation.x;
		marker_pose.at<float>(0, 4) = msg.orientation.y;
		marker_pose.at<float>(0, 5) = msg.orientation.z;
		newMarkerTime = msg.orientation.w;
		newMarker = true;
		cout << "New marker!" << endl;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cam1");

	ros::NodeHandle n("~");
	ros::Rate loop_rate(10);
	bool pubsubToggled = false;

	n.getParam("id", id);
	markerPose_sub = n.subscribe("/global/markerpose", 2, markerPoseCallback);
	Mat T;
	T.create(4, 4, CV_32FC1);
	marker_pose.create(3, 1, CV_32FC1);

	bool pitchStable = false;
	// Initialise all global variables.

	marker_pose = Mat::zeros(6, 1, CV_32FC1);

	config.captureInfo = new TrackerHelper::CaptureInfo;
	config.cameraInfo = new TrackerHelper::CameraInfo;
	config.marker.marker_pose = Mat::zeros(6, 1, CV_32FC1);
	config.marker.Tm = Mat::eye(4, 4, CV_32FC1);
	config.marker.mTime = -99999;

//  config.cameraInfo->camPose = Mat::zeros(6, 1, CV_32F);
	config.cameraInfo->camPose =(Mat_<float>(6, 1) << 	-1295.29197674419,	-30.1675683662791,	398.303308139535,	-0.363636244186046	,0.0483828476744186,	0.104771943604651);

	config.cameraInfo->Tc = Mat::eye(4, 4, CV_32F);
	config.cameraInfo->cameraPoseKnown = true;
	helper.initialiseCapture(id, config.captureInfo[0].capture);
	Mat frame;
	config.captureInfo[0].capture.read(frame);
	helper.initFrameSize(config.captureInfo[0].capture,
			config.captureInfo[0].width, config.captureInfo[0].height);
	config.captureInfo[0].newResult = false;

	String calFile = "/home/parallels/catkin_ws/src/more_t2/data/cam1/all.cal";
	helper.configTracker(config.captureInfo[0].width,
			config.captureInfo[0].height, calFile.c_str());

	IplImage* img;
	Mat temp;
	config.captureInfo[0].capture.read(temp);
	img = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U,
			temp.channels());

	string path;
	if (config.cameraInfo->cameraPoseKnown) {
		path = "/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_1.csv";
	} else {
		path = "/home/parallels/catkin_ws/src/more_t2/posedata/cam.csv";
	}
	ofstream fp;
	fp.open(path.c_str(), ios::out | ios::trunc);
	if (!fp.is_open()) {
		cout << "Error! Cannot open file to save pose data" << endl;
		exit(EXIT_FAILURE);
	}
	int count = 0;

	while (ros::ok()) {

		// Debug prints.
		Mat temp;
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		frameTime = config.captureInfo[0].capture.get(CV_CAP_PROP_POS_MSEC);
		if (temp.empty()) {
			ROS_INFO("ERROR: files to grab new image\n");
			exit(EXIT_FAILURE);
		}

		while ((newMarkerTime - frameTime) > timeout) {
			config.captureInfo[0].capture.read(temp);
			frameTime = config.captureInfo[0].capture.get(CV_CAP_PROP_POS_MSEC);
			if (temp.empty()) {
				ROS_INFO("ERROR: files to grab new image\n");
				exit(EXIT_FAILURE);
			}
		}

		while ((!newMarker) & !config.cameraInfo[0].cameraPoseKnown) {
			ros::spinOnce();
		}

		img->imageData = (char*) temp.data;

		int numDetected = helper.getNumDetected(img, helper.tracker,
				config.captureInfo[0].width, config.captureInfo[0].height, id); // Get transformation matrix from new result.

		if (config.cameraInfo[0].cameraPoseKnown) {
			// Update PubSub
			// Shutdown subcriber if still on and start publisher if not started
			if (pubsubToggled == false) {
				markerPose_pub = n.advertise<geometry_msgs::Pose>(
						"/global/markerpose", 2);
				markerPose_sub.shutdown();
				pubsubToggled = true;
			}

			// If camera's position is known then start publishing marker position
			// publish marker pose.

			if (numDetected > 0) {
				config.marker.mTime = frameTime;
				cout << "Cam id: " << id << "Frame time: " << frameTime << endl;
				helper.calcMarkerPose(helper.tracker,
						config.cameraInfo[0].camPose, config.marker.marker_pose,
						config.marker.Tm);
				cout << "Marker Pose:" << endl << config.marker.marker_pose
						<< endl;
				//Publish results
				geometry_msgs::Pose pose;
				pose.position.x = config.marker.marker_pose.at<float>(0, 0);
				pose.position.y = config.marker.marker_pose.at<float>(0, 1);
				pose.position.z = config.marker.marker_pose.at<float>(0, 2);
				pose.orientation.x = config.marker.marker_pose.at<float>(0, 3);
				pose.orientation.y = config.marker.marker_pose.at<float>(0, 4);
				pose.orientation.z = config.marker.marker_pose.at<float>(0, 5);
				pose.orientation.w = frameTime; // Z stores marker time.
				markerPose_pub.publish(pose);

				//store marker in file
				bool invalidMarkerPose = false;
				for (int i = 0; i < 6; i++) {
					float test = config.marker.marker_pose.at<float>(i, 0);
					if (test != test) {
						invalidMarkerPose = true;
					} else if (abs(test) > 1e4) {
						invalidMarkerPose = true;
					}
				}
				if (!invalidMarkerPose) {
					for (int i = 0; i < 6; i++) {
						fp << config.marker.marker_pose.at<float>(i, 0) << ",";

					}
					fp << frameTime << "\n";
				}
			}
		} else { // Derive Camera's position from marker image.
			if (numDetected) {
				cout << "cam,  id: " << id << endl;
				helper.getCameraPose(helper.tracker, &marker_pose,
						&config.cameraInfo[0].camPose, config.marker.Tm,
						pitchGain);
				cout << "Camera Pose" << endl << config.cameraInfo[0].camPose
						<< endl;

				helper.calcMarkerPose(helper.tracker,
						config.cameraInfo[0].camPose, config.marker.marker_pose,
						config.marker.Tm);
				cout << "Global Marker Pose :" << endl << marker_pose << endl;
				cout << "Estimated Marker Pose:" << endl
						<< config.marker.marker_pose << endl;

				if (!pitchStable & !isInvalidPose(config.cameraInfo[0].camPose)
						& !isInvalidPose(config.marker.marker_pose)) {
					bool result = updatePitchGain(pastGain, pastAveErr, counter,
							pitchGain, marker_pose, config.marker.marker_pose);
					if (!result) {
						pitchStable = true;
					}
				}
				cout << "Pitch Gain " << pitchGain << endl;
				bool invalidMarkerPose = false;
				for (int i = 0; i < 6; i++) {
					float test = config.cameraInfo[0].camPose.at<float>(i, 0);
					if (test != test) {
						invalidMarkerPose = true;
					} else if (abs(test) > 1e9) {
						invalidMarkerPose = true;
					}
				}
				if ((!invalidMarkerPose) & pitchStable) {
					for (int i = 0; i < 6; i++) {
						fp << config.cameraInfo[0].camPose.at<float>(i, 0);
						if (i == 5) {
							fp << "\n";
						} else {
							fp << ",";
						}
					}
					count++;
					cout << "Count: " << count << endl;
				}
			}
		}
		// Update pub sub

		// If this camera is calibrated, then send the position of the marker as ROS message.
		// To do this, first tracker marker and report its position.

		// If camera's pose is not known then we need to check for new marker positions.
		if (!config.cameraInfo->cameraPoseKnown & (count == 200)) {
			fp.close();
			config.captureInfo[0].capture.release();
			break;
		}
		loop_rate.sleep();
	}
	fp.close();
	config.captureInfo[0].capture.release();
}
