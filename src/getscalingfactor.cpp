#include "trackerhelper.h"
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;


TrackerHelper helper;
int id  = 0;
int samples = 50;
bool scaled = false;
float scaling = 1;

int main(int argc, char** argv) {

	float acc[samples];
	// Generate cam_pose array based on number of cameras and referenced by ID;

	// Generate tracking information for all relevant camera feeds.
	//. input arguments are: id, ip, distance
	if (argc != 4) {
		cout << "Invalid number of arguments" << endl;
		exit(1);
	}
	id = atoi(argv[1]);
	string ip = argv[2];
	int distance = atoi(argv[3]);

	timestamp_t timestamp;
	Mat frame;
	VideoCapture cap;

	// stringstream buffer;
	// // Todo: Change to right format.
	// buffer << "rtsp://admin:adminadmin@192.168.10.3" << id+1 << ":554/live.sdp";
	cap.open(ip.c_str());
	printf("Opening calibration video number : %d \n", id);
	if (!cap.isOpened()) {
		std::cout << "Could not initialize capturing...\n" << std::endl;
		exit(EXIT_FAILURE);
	}

		cap.read(frame);
	// Initialise capturing device.
	// We assume frames are all of the same size and are all gray scale.

	int width, height;
	// Initiliase width and heigth.
	helper.initFrameSize(frame, width, height);

	// Get the camera's pose

	stringstream cal;
	cal <<"../main/data/trendnet_IP310PI_Cam" << id+1<<".cal";
	if (helper.configTracker(width, height, cal.str().c_str()) != 0) {
		cout << "Error! Cannot configure tracker." << endl;
		exit(EXIT_FAILURE);
	}

	Mat pose;
	pose.create(3, 1, CV_32F);
	Mat T;
	T.create(4, 4, CV_8UC(2));

	Mat tempImg;
	cap.read(tempImg);
	timestamp = cap.get(CV_CAP_PROP_POS_MSEC);

	if (tempImg.empty()) {
		printf("End of File!");
		exit(EXIT_FAILURE);
	}

	IplImage* img = cvCreateImage(cvSize(tempImg.cols, tempImg.rows),
	IPL_DEPTH_8U, tempImg.channels());

	// Todo: While video is not empty
	int index = 0;
	while (1) {
		cap.read(tempImg);
			timestamp = cap.get(CV_CAP_PROP_POS_MSEC);
		if (tempImg.empty()) {
			printf("End of File!");
			break;
		}
		img->imageData = (char *) tempImg.data;
		int newResult = helper.getNumDetected(img, helper.tracker, width,
				height, id);
		cout << "Frame time: " << timestamp << endl;
		tempImg.release();
		if (newResult==2) {
			Mat pose1, pose2;
			pose1.create(3, 0, CV_32F);
			pose2.create(3, 0, CV_32F);
			for (int i = 0; i < newResult; i++) {

				helper.getMarkerPosition(helper.tracker, i, pose);

				bool invalidMarkerPose = false;
				bool tempBool = true;
				for (int i = 0; i < 6; i++) {
					float test = pose.at<float>(i, 0);
					if (test != test) {
						invalidMarkerPose = true;
						break;
					} else if (abs(test) > 1e8) {
						invalidMarkerPose = true;
						break;
					}
					if (i < 3)
					if (test > 0.001) {
						tempBool = false;
						break;
					}
				}
				//check for all zeros
				invalidMarkerPose |= tempBool;
				if (!invalidMarkerPose) {
					cout << "New Marker" << endl;
					if (i == 0) {
						pose.copyTo(pose1);
					} else {
						pose.copyTo(pose2);
						acc[index++] = scaling = distance/ norm(pose1 - pose2);
						cout << "Count: " << index << endl << "pose1 " <<endl <<  pose1 << endl << "pose2 "<< endl << pose2 << endl;

						cout << "Scaling: " << scaling << endl;
						pose1.release();
						pose2.release();
					}
				}
			}
		}
		if (index == samples) {
			float sum = 0;
			for (int i = 0; i < samples; i++) {
				sum +=acc[i];
			}
			scaling = sum / samples;
			ofstream fp;
			stringstream buffer;
			buffer << "../main/cfg/scaling" << id;
			fp.open(buffer.str().c_str(), ios::out | ios::trunc);
			if (!fp.is_open()) {
				cout << "Error! Cannot open file to save scaling" << endl;
				exit(EXIT_FAILURE);
			}
			cout << "Final Scaling: " << scaling << endl;
			fp << scaling;
			fp.close();
			break;
		}
	}
	cap.release();
	return 0;
}


