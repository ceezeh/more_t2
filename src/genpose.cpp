#include "trackerhelper.h"

using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

float NONUMBER = 1e20;
bool isValid(Mat pose) {
	bool nonzero = false;
	for (int i = 0; i < 6; i++) {
		float test = pose.at<float>(i, 0);
		if (test != test) {
			return false;
		} else if (abs(test) > 1e8) {
			return false;
		} else if (test != 0) {
			nonzero =true;
		}

	}		
	return true && nonzero;		
}


float scalingfactor = 1;

int markerCount = 1;
bool defaultResult = true;
ofstream fp;



void fnExit1 (void)
{
	fp.close();
}


int main(int argc, char** argv) {
	atexit (fnExit1);

	TrackerHelper helper;
	string path, ar_configpath;
	Mat campose = (Mat_<float>(8,1) << 0, 0, 0, 0, 0, 0, 1, 1);
	vector<int> markerIDs;
	markerIDs.push_back(-1);
	string calib_name;
	int markersize;
	vector<Mat> pose;
	Mat temp = Mat::zeros(8, 1, CV_32F);
					pose.push_back(temp);
					
	Mat NONPOSE = (Mat_<float>(8, 1) << NONUMBER, NONUMBER, NONUMBER, NONUMBER, NONUMBER, NONUMBER, NONUMBER, NONUMBER); 
	



	char opt;

	while ((opt = getopt(argc, argv, "c:l:m:p:s:a:")) != -1) {
		switch (opt) {
			case 'c':
			{
				calib_name = optarg;
				cout << "Calibration here: "<< optarg <<endl;
				break;
			}
			case 'p':
			{
				string campose_str = optarg;
				stringstream buffer(campose_str);
				float x,y,z,u,v,w,ys,zs;
				buffer >> x;
				buffer>>y>>z>>u>>v>>w>>ys>>zs;
				campose = (Mat_<float>(8,1) << x,y,z,u,v,w,ys,zs);
				// cout << "x,y,z,u,v,w,ys,zs: "<< x<< " "<< y<< " "<<z<< " "<<u<<" "<<v<<" "<<w<<" "<<ys<<" "<<zs<<endl;
				cout << "Campose: " << endl << campose << endl;
				break;
			}
			case 'l':
			{
				path = optarg;
				cout << "data path : "<<path<<endl;
				break;
			}
			case 'm':
			{
				string markerID_str = optarg;
				stringstream buffer(markerID_str);
				cout << "marker id str: "<< markerID_str << endl;
				markerIDs.clear();
				pose.clear();
				int id;
				while (!buffer.eof()){
					buffer >> id;
					markerIDs.push_back(id);
					Mat temp = Mat::zeros(8, 1, CV_32F);
					pose.push_back(temp);
				}
				//sort in ascending order.
				std::sort (markerIDs.begin(), markerIDs.end());
				break;
			}
			case 's':
				markersize = atoi(optarg);
				cout << "Marker Size: "<< markersize << endl;
				helper.setMarkerWidth(markersize);
				break;
			case 'a':
				ar_configpath = optarg;
		}
	}

	// Location of Calibration:
		string calibpath = path + "/ar_calibration.cal";
		cout << "calib path : "<<calibpath<<endl;


	TrackerHelper::ImageReader imReader(path);
	timestamp_t timestamp;
	Mat frame;
	imReader.vidRead(frame, timestamp);
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

	
		// open results storage file and marker size file.
	string resultPath = path + "/pose.csv";

	cout << "resultPath" << endl << resultPath << endl;
//		cout << "id" << endl << id << endl;
		// File stream to save data.

	fp.open(resultPath.c_str(), ios::out | ios::trunc);
	
	if (!fp.is_open()) {
		cout << "Error! Cannot open file to save pose data" << endl;
		exit(EXIT_FAILURE);
	}

	int markerCount = markerIDs.size();
	cout << "markerCount: " << markerIDs.size() << endl;
	bool processed[markerCount];
	for (int i = 0; i < markerCount; i++) {
		processed[i] = false;
	}

		// write the header for the file.
	for (int i = 0; i <  markerCount; i++) {
		stringstream prefix;
		prefix << markerIDs[i] << "_";
		cout << "  markerIDs: " << markerIDs[i] << endl;
		fp << prefix.str() << "X"<< "," 
		<< prefix.str() << "Y" << "," 
		<< prefix.str() << "Z" << ","
		<< prefix.str() << "Pitch"<< ","
		<< prefix.str() << "Yaw" << ","
		<< prefix.str() << "Roll"<< ","
		<< prefix.str() << "Ys"<<","
		<< prefix.str() << "Zs" <<((i == (markerCount-1)) ? ",Timestamp\n" : ",");
	}	 


	Mat tempImg;
	imReader.vidRead(tempImg, timestamp);

	if (tempImg.empty()) {
		printf("End of File!");
		exit(EXIT_FAILURE);
	}

	IplImage* img = cvCreateImage(cvSize(tempImg.cols, tempImg.rows),
		IPL_DEPTH_8U, tempImg.channels());

		// Todo: While video is not empty
	while (true) {
		imReader.vidRead(tempImg, timestamp);
		if (tempImg.empty()) {
			printf("End of File!\n");
			break;
		}
		img->imageData = (char *) tempImg.data;
		int newResult = helper.getNumDetected(img, helper.tracker, width,
			height, (char *)calib_name.c_str());
		cout << newResult << " marker(s) detected at frame time: " << timestamp << endl;
		tempImg.release();

		if (markerCount >1) {
			std::vector<int>::iterator it;
				// reinitialise marker process flags.
			for (int i = 0; i < markerCount; i++) {
				processed[i] = false;
			}
			int counter = newResult -1;
			while (counter > -1) {
				int markerID = helper.tracker->getDetectedMarker(counter).id;

				it = find (markerIDs.begin(), markerIDs.end(), markerID);
					// flag that marker has been processing in this frame.
				if (it != markerIDs.end()) {
					int index = distance( markerIDs.begin(), it );
					cout << "Marker Index: " << index << endl;
					processed [index] = true;

					
					helper.get6DOFMarkerPose(helper.tracker, campose, pose[index], counter);
					cout << "pose1 :" << endl<<pose[index]<<endl;
					if (!isValid(pose[index])) {
						NONPOSE.copyTo(pose[index]);
					}
					cout << "pose2 :" << endl<<pose[index]<<endl;

				}
				counter --;
			}
				// Now write all to file.
			for (int i = 0; i < markerCount; i++) {

				if (processed[i]) {
					stringstream str;
					str << "," << timestamp << "\n";

					fp << pose[i].at<float>(0, 0) << "," 
					<< pose[i].at<float>(1, 0) << "," 
					<< pose[i].at<float>(2, 0) << ","
					<< pose[i].at<float>(3, 0) << ","
					<< pose[i].at<float>(4, 0) << ","
					<< pose[i].at<float>(5, 0) << ","
					<< pose[i].at<float>(6, 0) << ","
					<< pose[i].at<float>(7, 0)
					<< ((i == (markerCount-1)) ? str.str() : ",");
				} else {
					stringstream str;
					str << "," << timestamp << "\n";
					fp  << NONUMBER << "," 
					<< NONUMBER << "," 
					<< NONUMBER << ","
					<< NONUMBER << ","
					<< NONUMBER << ","
					<< NONUMBER << ","
					<< NONUMBER << ","
					<< NONUMBER
					<< ((i == (markerCount-1)) ? str.str() : ",");
				}

			}
		} else {
			if (newResult>0) {

				cout << "check pose:" << endl << pose[0]<<endl;
				helper.get6DOFMarkerPose(helper.tracker, campose, pose[0], 0);

					//check if data is valid
				cout << "marker pose " << endl<< pose[0];
				if (isValid(pose[0])) {
					cout << "New Marker" << endl;

					fp << pose[0].at<float>(0, 0) << "," 
					<< pose[0].at<float>(1, 0) << "," 
					<< pose[0].at<float>(2, 0) << ","
					<< pose[0].at<float>(3, 0) << ","
					<< pose[0].at<float>(4, 0) << ","
					<< pose[0].at<float>(5, 0) << ","
					<< pose[0].at<float>(6, 0) << ","
					<< pose[0].at<float>(7, 0) << ","<< timestamp << "\n";
				}

			}
		}

	}
	for (int i = 0; i < markerCount; i++) {
		pose[i].release();
	}
	fp.close();
	return 0;
}


