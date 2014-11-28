#include "trackerhelper.h"
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;


TrackerHelper helper;

int main(int argc, char** argv) {

	ros::init(argc, argv, "more_t2");

	ros::NodeHandle n("~");
	ros::Rate loop_rate(10);

	int noCams = 2;
	n.getParam("noCams", noCams); //The total number of possible cameras.
	// Generate cam_pose array based on number of cameras and referenced by ID;

	// Generate tracking information for all relevant camera feeds.
	for (int id = 0; id < noCams; id++) {

		// Initialise capturing device.
		// We assume frames are all of the same size and are all gray scale.

		int width, height;
		VideoCapture capture;
		helper.initialiseCapture(id, capture);
		//Initiliase width and heigth.
		helper.initFrameSize(capture, width, height);

		// Get the camera's pose
		Mat cam_pose;
//		helper.getCamPose(&cam_pose, id);
		//Todo:remove this for release!!
		if (id == 0) {
			cam_pose = Mat::zeros(6,1,CV_32FC1);
		} else if (id == 1){
//			cam_pose = (Mat_<float>(6, 1) <<  986.1998, 13.9150, 1034.5, -0.1350, -0.4107,  0.1685);
			cam_pose = (Mat_<float>(6, 1) <<  986.1998, 13.9150, 1034.5, -0.1350, -0.4107,  0.1685);
		}
		if (helper.configTracker(width, height) != 0) {
			cout << "Error! Cannot configure tracker." << endl;
			exit(EXIT_FAILURE);
		}

		Mat pose;
		pose.create(6, 1, CV_32F);
		Mat T;
		T.create(4, 4, CV_8UC(2));

		stringstream sbuffer;
		string path = "/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_";
		sbuffer << path << id << ".csv";
//		cout << "sbuffer" << endl << sbuffer.str() << endl;
//		cout << "id" << endl << id << endl;
		// File stream to save data.
		ofstream fp;
		fp.open(sbuffer.str().c_str(), ios::out | ios::trunc);
		sbuffer.str("");
		if (!fp.is_open()) {
			cout << "Error! Cannot open file to save pose data" << endl;
			exit(EXIT_FAILURE);
		}
		Mat tempImg;
		capture.read(tempImg);

		if (tempImg.empty()) {
			printf("End of File!");
			break;
		}
		IplImage* img = cvCreateImage(cvSize(tempImg.cols, tempImg.rows),
		IPL_DEPTH_8U, tempImg.channels());

		// Todo: While video is not empty
		while (ros::ok()) {
			capture.read(tempImg);
			capture.read(tempImg);
				capture.read(tempImg);
				capture.read(tempImg);
				capture.read(tempImg);
				capture.read(tempImg);
								capture.read(tempImg);
								capture.read(tempImg);
								capture.read(tempImg);
								capture.read(tempImg);
			int frameTime = capture.get(CV_CAP_PROP_POS_MSEC);
			if (tempImg.empty()) {
				printf("End of File!");
				break;
			}
			img->imageData = (char *) tempImg.data;
			bool newResult = helper.processMarkerImg(img, helper.tracker, width,
					height, id);
			cout << "Frame time: " << frameTime << endl;
			tempImg.release();
			if (newResult) {
				helper.calcMarkerPose(helper.tracker, cam_pose, pose, T);
				// Todo: Implement Constant frame rate!
				// Save pose to file with  time stamp.
				// To do this, increment each time based on frame rate.
				// Here we assume frame rate is constant as given by a hardware trigger.

				// We save each row in the form, x, y, z, t;
				// Todo: Save information on 6DOF trajectory so include:
				// Yaw, Pitch and Row information.
				//check if data is valid
				bool invalidMarkerPose = false;
				for (int i = 0; i < 6; i++) {
					float test = pose.at<float>(i, 0);
					if (test != test) {
						invalidMarkerPose = true;
					} else if (abs(test) > 1e4) {
						invalidMarkerPose = true;
					}
				}
				if (!invalidMarkerPose) {
					fp << pose.at<float>(0, 0) << "," << pose.at<float>(1, 0)
							<< "," << pose.at<float>(2, 0) << ","
							<< pose.at<float>(3, 0) << ","
							<< pose.at<float>(4, 0) << ","
							<< pose.at<float>(5, 0) << "," << frameTime << "\n";
				}
			}
			// Todo: Scale time by frame rate factor.

			ros::spinOnce();
			loop_rate.sleep();
		}
		capture.release();
		fp.close();
	}

	return 0;
}
