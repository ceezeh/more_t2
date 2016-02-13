#include "trackerhelper.h"
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;


TrackerHelper helper;
int samples = 50;
Mat canvas;
int id = 0;
float scaling = 1;
bool scaled = false;


int main(int argc, char** argv) {




	string path,ip,calib_name,ar_configpath;
	int markersize;
	char opt;

	while ((opt = getopt(argc, argv, "l:s:a:i:")) != -1) {
		switch (opt) {
			case 'l':
			{
				path = optarg;
				cout << "data path : "<<path<<endl;
				break;
			}
			case 's':
			markersize = atoi(optarg);
			cout << "Marker Size: "<< markersize << endl;
			helper.setMarkerWidth(markersize);
			break;
			case 'a':
			ar_configpath = optarg;
			break;
			case 'i':
			ip =optarg;
			cout << "address :"<< ip << endl; 
		}
	}

	string calibpath = path + "/ar_calibration.cal";
	cout << "calib path : "<<calibpath<<endl;



	timestamp_t timestamp;
	Mat frame;
	VideoCapture cap;

	// string file = "rtsp://admin:adminadmin@192.168.10.34:554/live.sdp";
	// 	stringstream buffer;
	// 	// Todo: Change to right format.
	// 	buffer << "rtsp://admin:adminadmin@192.168.10.3" << id+1 << ":554/live.sdp";
	cap.open(ip.c_str());
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	printf("Opening calibration video number : %d", id);
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

	Mat tempMat;
	cap.read(tempMat);
	timestamp = cap.get(CV_CAP_PROP_POS_MSEC);

	if (tempMat.empty()) {
		printf("End of File!");
		exit(EXIT_FAILURE);
	}

	IplImage* img = cvCreateImage(cvSize(tempMat.cols, tempMat.rows),
		IPL_DEPTH_8U, tempMat.channels());

	// Todo: While video is not empty
	while (1) {
		cap.read(tempMat);
		timestamp = cap.get(CV_CAP_PROP_POS_MSEC);
		if (tempMat.empty()) {
			printf("End of File!");
			break;
		}
		img->imageData = (char *) tempMat.data;

		int newResult = 0;
		IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

		IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
		cvCvtColor(img, tempImg, CV_RGB2GRAY);
		cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
			CV_THRESH_BINARY, 171);
		newResult = helper.tracker->calc((unsigned char*) greyImg->imageData);
		char name[10];


		cout <<" No of markers: " << newResult << endl;
		tempMat.release();
		while (newResult>-1) {

			ARToolKitPlus::ARMarkerInfo marker = helper.tracker->getDetectedMarker(newResult);

			cvCircle( img, cvPoint( int(marker.pos[0]),int(marker.pos[1]) ), 30, CV_RGB(100, 100, 255 ), -1 );
			newResult--;
		}

		sprintf(name, "cam: %d", id);
		cvShowImage(name, img);
	cvWaitKey(2); // Wait for image to be rendered on screen. If not included, no image is shown.
	
	cvReleaseImage(&greyImg);
	cvReleaseImage(&tempImg);
}
cap.release();
return 0;
}


