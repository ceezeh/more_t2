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
//  const ros::Duration timeout(0, 100000000); // 100 milli second.
	int noCams = 2;

	n.getParam("noCams", noCams);

	Mat T;
	T.create(4, 4, CV_32FC1);
	TrackerHelper::TransGraph transGraph;
	TrackerHelper::RotGraph rotGraph;
	// Initialise all global variables.

	float timeout = 100; //ms
	TrackerHelper::Config config;
	config.captureInfo = new TrackerHelper::CaptureInfo[noCams];
	config.cameraInfo = new TrackerHelper::CameraInfo[noCams];
	config.marker.marker_pose = Mat::zeros(6, 1, CV_32F);

	for (int i = 0; i < noCams; i++) {
		config.cameraInfo[i].camPose = Mat::zeros(6, 1, CV_32F);
		config.cameraInfo[i].T = Mat::zeros(4, 4, CV_32F);
		config.cameraInfo[i].cameraPoseKnown = false;
		helper.initialiseCapture(i, config.captureInfo[i].capture);
		Mat frame;
		//Todo: May need to make width and height global like tracker
		config.captureInfo[i].capture.read(frame);
		helper.initFrameSize(config.captureInfo[i].capture,
				config.captureInfo[i].width, config.captureInfo[i].height);
//    configTracker(config.captureInfo[i].tracker, config.captureInfo[i].width, config.captureInfo[i].height);
		config.captureInfo[i].newResult = false;
	}
	helper.configTracker(config.captureInfo[0].width,
			config.captureInfo[0].height);

// Initialise known camera's position.
	stringstream stream;
	string pose_s;
	int x, y, z, roll, yaw, pitch;
	n.getParam("pose", pose_s);
	stream << pose_s;
	stream >> x >> y >> z >> roll >> yaw >> pitch;
	config.cameraInfo[0].camPose =
			(Mat_<float>(6, 1) << x, y, z, roll, yaw, pitch);
	// Todo:This is for debug only.
	config.cameraInfo[0].camPose = Mat::zeros(6, 1, CV_32F);
	config.cameraInfo[0].cameraPoseKnown = true;
	helper.writeCalibrationToFile(0, config.cameraInfo[0].camPose);

	IplImage* imgArr[noCams];
	int dataSize = 200;
	float stats[6][dataSize];
	int count[noCams];
	int preCount[noCams];
	for (int id = 0; id < noCams; id++) {
		Mat temp;
		config.captureInfo[id].capture.read(temp);
		imgArr[id] = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U,
				temp.channels());
		count[id]=0;
		preCount[id]=0;
	}
	Mat samples(dataSize, 6, CV_32F);
	while (ros::ok()) {

		for (int id = 0; id < noCams; id++) {

			Mat temp;
			config.captureInfo[id].capture.read(temp);
      int frameTime = config.captureInfo[id].capture.get(CV_CAP_PROP_POS_MSEC);
//			int frameTime = -1;
//      cout << "Frame time: " << frameTime << " msecs; id: "<<id<< endl;
			if (temp.empty()) {
				ROS_INFO("ERROR: files to grab new image\n");
				exit(EXIT_FAILURE);
			}

			imgArr[id]->imageData = (char*) temp.data;

			config.captureInfo[id].newResult = helper.processMarkerImg(
					imgArr[id], helper.tracker, config.captureInfo[id].width,
					config.captureInfo[id].height, id); // Get transformation matrix from new result.

			if (config.cameraInfo[id].cameraPoseKnown) {
				// If camera's position is known then start publishing marker position
				// publish marker pose.
//        cout << "cam_pose" << endl << config.cameraInfo[id].camPose << endl << endl;
				if (config.captureInfo[id].newResult) {
					config.marker.mTime = frameTime;
					cout <<"Start! id: " << id<< endl;
					helper.calcMarkerPose(helper.tracker,
							config.cameraInfo[id].camPose,
							config.marker.marker_pose, config.cameraInfo[id].T);
//          drawOrientation(config.marker.marker_pose);
//          drawTrans(config.marker.marker_pose);
//          cout << "Marker Pose from id: " << id << endl << config.marker.marker_pose << endl << endl;
					config.captureInfo[id].newResult = false;
				}
			} else { // Derive Camera's position from marker image.
//        cout << "Unknown Cam Pose" << endl;

//				if (config.captureInfo[id].newResult)
          if (config.captureInfo[id].newResult && ((frameTime - config.marker.mTime) < timeout))
				{
//          cout << "Getting cam pose" << endl;
					helper.getCameraPose(helper.tracker,
							&config.marker.marker_pose,
							&config.cameraInfo[id].camPose,
							config.cameraInfo[id].T);
//          config.cameraInfo[id].cameraPoseKnown = true;
					config.captureInfo[id].newResult = false;
//          cout << "cam_pose: " << config.cameraInfo[id].camPose.at<float>(0,0) << "; marker: " << config.marker.marker_pose.at<float>(0,0) <<"id :" << id << endl << endl;
//          drawTrans(config.cameraInfo[id].camPose);
					//          cout << "; cam0-T:" << config.cameraInfo[0].T<<endl;// << "; cam1-T:" << config.cameraInfo[1].T.at<float>(3,0) << "; diff:"
//        		 << config.cameraInfo[0].T.at<float>(3,0) - config.cameraInfo[1].T.at<float>(3,0) << endl << endl;
					rotGraph.drawOrientation(config.cameraInfo[id].camPose, "Orientation");
					cout << "cam_pose,  id: " << id << endl
							<< config.cameraInfo[id].camPose << endl << endl;
					bool invalidCamPose = true;
					preCount[id] ++;
					if (preCount[id] >= 100) {
						invalidCamPose = false;
					}
					//check if data is valid
					for (int i = 0; i < 6; i++){
						float test = config.cameraInfo[id].camPose.at<float>(i, 0);
						if (test != test) {
							 invalidCamPose = true;
						}else	if (abs(test) > 1e4) {
							 invalidCamPose = true;
						}
					}
					if ((count[id] < dataSize)&(!invalidCamPose)) {
						// Collect sample
						Mat sample = samples.row(count[id]);
						((Mat)config.cameraInfo[id].camPose.t()).copyTo(sample);
						cout <<"sample" << endl<< samples.row(count[id]) << endl<<endl;
//						stats[0][count[1]] =
//								config.cameraInfo[id].camPose.at<float>(0, 0);
//						stats[1][count[1]] =
//								config.cameraInfo[id].camPose.at<float>(0, 1);
//						stats[2][count[1]] =
//								config.cameraInfo[id].camPose.at<float>(0, 2);
//						stats[3][count[1]] =
//								config.cameraInfo[id].camPose.at<float>(0, 3);
//						stats[4][count[1]] =
//								config.cameraInfo[id].camPose.at<float>(0, 4);
//						stats[5][count[1]] =
//								config.cameraInfo[id].camPose.at<float>(0, 5);
						count[id]++;
						cout << "id: "<<id<<"; Count: " << count[id] << endl << endl;
					} else if (count[id] == dataSize) {
						//cluster samples
						Mat centers;
						helper.clusterData(samples, centers);
						cout << "Please choose the most appropriate camera pose" << endl;
						int reply;
						cin >> reply;
						while ((cin.bad())|(reply<0)|(reply>=5)) {
							cout << "invalid choice" << endl;
							cout << "Please choose the most appropriate camera pose" << endl;
							cin >> reply;
						}
						config.cameraInfo[id].cameraPoseKnown = true;
						// Calculate std and mean.
//						float mean[6] = { 0, 0, 0, 0, 0, 0 };
//						float stdev[6] = { 0, 0, 0, 0, 0, 0 };
//						for (int i = 0; i < dataSize; i++) {
//							mean[0] += stats[0][i];
//							mean[1] += stats[1][i];
//							mean[2] += stats[2][i];
//							mean[3] += stats[3][i];
//							mean[4] += stats[4][i];
//							mean[5] += stats[5][i];
//						}
//						mean[0] /= dataSize;
//						mean[1] /= dataSize;
//						mean[2] /= dataSize;
//						mean[3] /= dataSize;
//						mean[4] /= dataSize;
//						mean[5] /= dataSize;
//						// get stdev
//						for (int i = 0; i < dataSize; i++) {
//							stdev[0] = pow((mean[0] - stats[0][i]), 2);
//							stdev[1] = pow((mean[1] - stats[1][i]), 2);
//							stdev[2] = pow((mean[2] - stats[2][i]), 2);
//							stdev[3] = pow((mean[3] - stats[3][i]), 2);
//							stdev[4] = pow((mean[4] - stats[4][i]), 2);
//							stdev[5] = pow((mean[5] - stats[5][i]), 2);
//						}
//						stdev[0] = sqrt(stdev[0] /= dataSize);
//						stdev[1] = sqrt(stdev[1] /= dataSize);
//						stdev[2] = sqrt(stdev[2] /= dataSize);
//						stdev[3] = sqrt(stdev[3] /= dataSize);
//						stdev[4] = sqrt(stdev[4] /= dataSize);
//						stdev[5] = sqrt(stdev[5] /= dataSize);
//						cout << "mean" << endl << "[" << mean[0] << ";" << endl
//								<< mean[1] << ";" << endl << mean[2] << ";"
//								<< endl << mean[3] << ";" << endl << mean[4]
//								<< ";" << endl << mean[5] << "]" << endl
//								<< endl;
//						cout << "stdev" << endl << "[" << stdev[0] << ";"
//								<< endl << stdev[1] << ";" << endl << stdev[2]
//								<< ";" << endl << stdev[3] << ";" << endl
//								<< stdev[4] << ";" << endl << stdev[5] << ";"
//								<< endl;
//						count++;
//						config.cameraInfo[id].cameraPoseKnown = true;
//						//Update camepose
//						config.cameraInfo[id].camPose.at<float>(0, 0) = mean[0];
//						config.cameraInfo[id].camPose.at<float>(0, 1) = mean[1];
//						config.cameraInfo[id].camPose.at<float>(0, 2) = mean[2];
//						config.cameraInfo[id].camPose.at<float>(0, 3) = mean[3];
//						config.cameraInfo[id].camPose.at<float>(0, 4) = mean[4];
//						config.cameraInfo[id].camPose.at<float>(0, 5) = mean[5];
//
						helper.writeCalibrationToFile(id,
								centers.row(reply).t());
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
			if (shut == true) {
				// release all capture devices.
				for (int i = 0; i < noCams; i++) {
					config.captureInfo[i].capture.release();
				}
				cout << "Shutting down!" << endl;
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
