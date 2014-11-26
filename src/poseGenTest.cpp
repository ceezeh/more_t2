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

	TrackerHelper::Config config;
	config.captureInfo = new TrackerHelper::CaptureInfo[noCams];
	config.cameraInfo = new TrackerHelper::CameraInfo[noCams];
	Mat markerPoses[noCams];
	ofstream fpArr[noCams];
	TrackerHelper::RotGraph rotGraphs[noCams];
	TrackerHelper::TransGraph transGraphs[noCams];
	for (int i = 0; i < noCams; i++) {
		config.cameraInfo[i].camPose = Mat::zeros(6, 1, CV_32F);
		config.cameraInfo[i].Tc = Mat::zeros(4, 4, CV_32F);
		helper.initialiseCapture(i, config.captureInfo[i].capture);
		Mat frame;
		//Todo: May need to make width and height global like tracker
		config.captureInfo[i].capture.read(frame);
		helper.initFrameSize(config.captureInfo[i].capture,
				config.captureInfo[i].width, config.captureInfo[i].height);
		config.captureInfo[i].newResult = false;
		// Get the camera's pose
		helper.getCamPose(&config.cameraInfo[i].camPose, i);
		markerPoses[i] = Mat::zeros(6, 1, CV_32F);


		// open file to save trajectory.
		stringstream sbuffer;
	    string path = "/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_";
	    sbuffer << path << i << ".csv";
	    cout << "sbuffer" << endl << sbuffer.str() << endl;
	    // File stream to save data.

	    fpArr[i].open(sbuffer.str().c_str(), ios::out | ios::trunc);
	    sbuffer.str("");
	    if (!fpArr[i].is_open())
	    {
	      cout << "Error! Cannot open file to save pose data" << endl;
	      exit(EXIT_FAILURE);
	    }
	}
	IplImage* imgArr[noCams];
	int t = 0;
	for (int id = 0; id < noCams; id++) {
			Mat temp;
			config.captureInfo[id].capture.read(temp);
			imgArr[id] = cvCreateImage(cvSize(temp.cols, temp.rows), IPL_DEPTH_8U,
					temp.channels());
		}

		helper.configTracker(config.captureInfo[0].width, config.captureInfo[0].height);

		while (ros::ok()) {
			for (int id = 0; id < noCams; id++) {
				Mat tempImg;
				config.captureInfo[id].capture.read(tempImg);
			if (tempImg.empty()) {
				printf("End of File!");
				break;
			}
			imgArr[id]->imageData = (char*) tempImg.data;
			bool newResult = helper.processMarkerImg(imgArr[id], helper.tracker, config.captureInfo[0].width,
					config.captureInfo[0].height, id);
			if (newResult) {

				helper.calcMarkerPose(helper.tracker, config.cameraInfo[id].camPose, markerPoses[id], config.cameraInfo[id].Tc);
				cout << "id: " << id << " marker pose" << endl << markerPoses[id] << endl <<endl ;
				// Todo: Implement Constant frame rate!
				// Save pose to file with  time stamp.
				// To do this, increment each time based on frame rate.
				// Here we assume frame rate is constant as given by a hardware trigger.

				// We save each row in the form, x, y, z, t;
				// Todo: Save information on 6DOF trajectory so include:
				// Yaw, Pitch and Row information.
//				fpArr[id] << markerPoses[id].at<float>(0,0) << "," << markerPoses[id].at<float>(1, 0) << ","
//						<< markerPoses[id].at<float>(2, 0) << "," << markerPoses[id].at<float>(3, 0)
//						<< "," << markerPoses[id].at<float>(4, 0) << ","
//						<< markerPoses[id].at<float>(5, 0) << "," << t << "\n";
				char transTitle[10];
				char rotTitle[10];
				sprintf(transTitle, "trans: %d", id);
				sprintf(rotTitle, "rot: %d", id);
				transGraphs[id].drawTrans(markerPoses[id], transTitle);
				rotGraphs[id].drawOrientation(markerPoses[id], rotTitle);
			}
			// Todo: Scale time by frame rate factor.


			ros::spinOnce();
			loop_rate.sleep();

		}
			t++;
		}
		for (int i =0; i<noCams;i++){
			config.captureInfo[i].capture.release();
			fpArr[i].close();
		}


	return 0;
}
