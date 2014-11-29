/*
 * camTest0.cpp
 *
 *  Created on: Nov 29, 2014
 *      Author: parallels
 */
#include "trackerhelper.h"
#include <geometry_msgs/PoseStamped.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

int id = 0;
int width;
int height;
bool cameraPoseKnown = false;
CvCapture* capture = NULL;
TrackerMultiMarker *tracker;
ros::Publisher markerPose_pub;
// The pose of the initial camera.
Mat marker_pose;

//Pylon variables.
// Automagically call PylonInitialize and PylonTerminate to ensure
// the pylon runtime system is initialized during the lifetime of this object.
//Debug variable
bool camInitialise = false;
bool newMarker = false;
int frameTime = -9999;
int timeout = 100;
ros::Subscriber markerPose_sub;

void initialisation() {
	// Initialise Capturing Device.

	initialiseCam(id);
	initFrameSize();
	configTracker();
}

// Updates and publishes marker pose.


void markerPoseCallback(const geometry_msgs::PoseStamped & msg) {
	if (abs(frameTime - msg.header.stamp) < timeout) {
		marker_pose.at<float>(0, 0) = msg.pose.position.x;
		marker_pose.at<float>(0, 1) = msg.pose.position.y;
		marker_pose.at<float>(0, 2) = msg.pose.position.z;
		marker_pose.at<float>(0, 3) = msg.pose.orientation.w;
		marker_pose.at<float>(0, 4) = msg.pose.orientation.x;
		marker_pose.at<float>(0, 5) = msg.pose.orientation.y;
		newMarker = true;
		cout << "New marker!" << endl;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cam1");

	ros::NodeHandle n("~");
	ros::Rate loop_rate(10);
	bool pubsubToggle = false;

	n.getParam("id", id);
	markerPose_sub = n.subscribe("/global/markerpose", 2, markerPoseCallback);
	Mat T;
	T.create(4, 4, CV_32FC1);
	marker_pose.create(3, 1, CV_32FC1);
	;
	//Pylon stuff

	// Initialise all global variables.

	Mat T;
	T.create(4, 4, CV_32FC1);
	// Initialise all global variables.

	marker_pose = Mat::zeros(6, 1, CV_32FC1);
	float timeout = 100; //ms
	TrackerHelper::Config config;
	config.captureInfo = new TrackerHelper::CaptureInfo;
	config.cameraInfo = new TrackerHelper::CameraInfo;
	config.marker.marker_pose = Mat::zeros(6, 1, CV_32FC1);
	config.marker.Tm = Mat::eye(4, 4, CV_32FC1);
	config.marker.mTime = -99999;

	config.cameraInfo->camPose = Mat::zeros(6, 1, CV_32F);
	config.cameraInfo->Tc = Mat::eye(4, 4, CV_32F);
	config.cameraInfo->cameraPoseKnown = false;
	helper.initialiseCapture(id, config.captureInfo[0].capture);
	Mat frame;
	config.captureInfo[0].capture.read(frame);
	helper.initFrameSize(config.captureInfo[0].capture,
			config.captureInfo[0].width, config.captureInfo[0].height);
	config.captureInfo[0].newResult = false;
	helper.configTracker(config.captureInfo[0].width,
			config.captureInfo[0].height);

	config.cameraInfo[0].cameraPoseKnown = true;

	IplImage* img;
	Mat temp;
	config.captureInfo[id].capture.read(temp);
	img = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U,
			temp.channels());

	while (ros::ok()) {
		// Debug prints.
		Mat temp;
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		config.captureInfo[0].capture.read(temp);
		frameTime = config.captureInfo[0].capture.get(
				CV_CAP_PROP_POS_MSEC);
		if (temp.empty()) {
			ROS_INFO("ERROR: files to grab new image\n");
			exit(EXIT_FAILURE);
		}



		img->imageData = (char*) temp.data;

		int numDetected = helper.getNumDetected(img, helper.tracker,
				config.captureInfo[id].width, config.captureInfo[id].height,
				id); // Get transformation matrix from new result.

		if (cameraPoseKnown) {
			// Update PubSub
			// Shutdown subcriber if still on and start publisher if not started
			if (pubsubToggle == false) {
				markerPose_pub = n.advertise<geometry_msgs::PoseStamped>(
						"/global/markerpose", 2);
				markerPose_sub.shutdown();
				pubsubToggle = true;
			}

			// If camera's position is known then start publishing marker position
			// publish marker pose.

			if (numDetected > 0){
				config.marker.mTime = frameTime;
				cout << "Cam id: " << id << endl;
				helper.calcMarkerPose(helper.tracker, config.cameraInfo[id].camPose, config.marker.marker_pose,
														config.marker.Tm);
				cout << "Marker Pose:"<< endl << config.marker.marker_pose << endl;
				//Publish results
				geometry_msgs::PoseStamped pose;
					pose.header.stamp = frameTime;
					pose.pose.position.x = config.marker.marker_pose.at<float>(0, 0);
					pose.pose.position.y = config.marker.marker_pose.at<float>(0, 1);
					pose.pose.position.z = config.marker.marker_pose.at<float>(0, 2);
					pose.pose.orientation.w = config.marker.marker_pose.at<float>(0, 3);
					pose.pose.orientation.x = config.marker.marker_pose.at<float>(0, 4);
					pose.pose.orientation.y = config.marker.marker_pose.at<float>(0, 5);
					markerPose_pub.publish(pose);

			}
		} else { // Derive Camera's position from marker image.
			if (numDetected & newMarker) {
				cout << "cam,  id: " << id<< endl;
								helper.getCameraPose(helper.tracker, &config.marker.marker_pose,&config.cameraInfo[id].camPose,
										config.marker.Tm);
								cout << "Camera Pose" << endl<<config.cameraInfo[id].camPose << endl;
								helper.calcMarkerPose(helper.tracker, config.cameraInfo[id].camPose, config.marker.marker_pose,
														config.marker.Tm);
								cout << "Marker Pose:"<< endl << config.marker.marker_pose << endl;
			}
		}
		// Update pub sub

		// If this camera is calibrated, then send the position of the marker as ROS message.
		// To do this, first tracker marker and report its position.

		// If camera's pose is not known then we need to check for new marker positions.

		ros::spinOnce();

		loop_rate.sleep();
	}
}
