#include "trackerhelper.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;


TrackerHelper helper;
int samples = 50;
int id = 0;


int main(int argc, char** argv) {




string path,ip,calib_name,ar_configpath;
	int markersize;
	char opt;

		float acc[6][samples];
	int tempPtr = 0;
	int index = 0;

	while ((opt = getopt(argc, argv, "l:s:a:i:")) != -1) {
		switch (opt) {
			case 'l':
			{
				path = optarg;
				// cout << "data path : "<<path<<endl;
				break;
			}
			case 's':
			markersize = atoi(optarg);
			// cout << "Marker Size: "<< markersize << endl;
			helper.setMarkerWidth(markersize);
			break;
			case 'a':
			ar_configpath = optarg;
			break;
			case 'i':
			ip =optarg;
			// cout << "address :"<< ip << endl; 
		}
	}

	string calibpath = path + "/ar_calibration.cal";
	// cout << "calib path : "<<calibpath<<endl;

	int fd;
    char * myfifo = "/tmp/adjustcampose";
    /* create the FIFO (named pipe) */
    mkfifo(myfifo, 0666);
   fd = open(myfifo, O_WRONLY);


	timestamp_t timestamp;
	Mat frame;
	VideoCapture cap;

		cap.open(ip.c_str());
		// printf("Opening calibration video number : %d", id);
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

	if (helper.configTracker(width, height, calibpath.c_str(),ar_configpath.c_str()) != 0) {
		cout << "Error! Cannot configure tracker." << endl;
		exit(EXIT_FAILURE);
	}

	Mat pose;
	pose.create(8, 1, CV_32F);
	Mat T;
	T.create(4, 4, CV_8UC(2));


	Mat tempImg;
	cap.read(tempImg);
	timestamp = cap.get(CV_CAP_PROP_POS_MSEC);

	if (tempImg.empty()) {
		// printf("End of File!");
		exit(EXIT_FAILURE);
	}

	IplImage* img = cvCreateImage(cvSize(tempImg.cols, tempImg.rows),
	IPL_DEPTH_8U, tempImg.channels());
	bool done = false;
	// Todo: While video is not empty
	while (1) {
		if (!done){
			float data[2] = {-99,-99};
			write(fd, data, 2*sizeof(float));
			usleep(100000);
		}
		cap.read(tempImg);
			timestamp = cap.get(CV_CAP_PROP_POS_MSEC);
		if (tempImg.empty()) {
			// printf("End of File!");
			break;
		}

		img->imageData = (char *) tempImg.data;

		int newResult = helper.getNumDetected(img, helper.tracker, width,
				height, "Cam Pose");
		cout << "Frame time: " << timestamp << endl;
		tempImg.release();
		if (newResult>0) {
			helper.getCameraPose(helper.tracker, &pose);
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
				cout << "New Marker" << endl;
				// fp << pose.at<float>(0, 0) << "," << pose.at<float>(1, 0)
				// 		<< "," << pose.at<float>(2, 0) << ","
				// 		<< pose.at<float>(3, 0) << ","
				// 		<< pose.at<float>(4, 0) << ","
				// 		<< pose.at<float>(5, 0) << ","
				// 		<< pose.at<float>(6, 0) << ","
				// 		<< pose.at<float>(7, 0) << ","<< timestamp << "\n";

				// Compute and display moving average. n = 100.
				if (tempPtr < samples) {
					for (int j = 0; j < 6; j++) {
						acc[j][tempPtr] = pose.at<float>(j, 0);
					}
					tempPtr++;
					cout << "tempptr: " << tempPtr << endl;
				} else {
					// summation;
					float sum[6] = {0,0,0,0,0,0};
					for (int i = 0; i < samples; i++) {
						for (int j = 0; j < 6; j++) {
							sum[j] += acc[j][i];
						}
					}

					//Update samples.
					// Wraparound.
					index = ((index + 1) == samples)? 0: index+1;

					for (int j = 0; j < 6; j++) {
						acc[j][index] = pose.at<float>(j, 0);
						sum[j] /= samples;
					}
					float pitch = sum[3]*180/CV_PI;
					float yaw = sum[4]*180/CV_PI;
					float data[2] = {pitch,yaw};
					done = true;
					write(fd, data, 2*sizeof(float));
		 			usleep(100000);

				}
			}
		}
		// Todo: Scale time by frame rate factor.

	}
	cap.release();
	close(fd);
	// fp.close();
	return 0;
}


