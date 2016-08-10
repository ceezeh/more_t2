/*
 * helper.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: parallels
 */
#include "trackerhelper.h"
 
int minArea = 9999;
void TrackerHelper::initialiseCapture(int id, VideoCapture &cap) {
	// Write a function to open camera
	// we will use an OpenCV capture
	string file =
			"../images_";
	stringstream buffer;
	// Todo: Change to right format.
	buffer << file << id << "/vid.wmv";
	cap.open(buffer.str().c_str());
//	cap.open(id);
	// cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	// cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	printf("Opening calibration video number : %d", id);
	if (!cap.isOpened()) {
		std::cout << "Could not initialize capturing...\n" << std::endl;
		exit(EXIT_FAILURE);
	}
}

void TrackerHelper::initFrameSize(Mat frame, int &width, int &height) {
	// Write a function to open camera

	if (frame.empty()) {
		printf("Failed to open image from capture device");
		exit(EXIT_FAILURE);
	}
	width = frame.cols;
	height = frame.rows;
}


int TrackerHelper::configTracker(int width, int height, const char* cal, const char* cfg ) {
	// write a function get camera frame size.
	tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

	// load a camera file.
	if (!tracker->init(cal,	cfg, 1.0f, 1000.0f))
			{
		printf("ERROR: init() failed\n");
		exit(EXIT_FAILURE);
	}

	// tracker->getCamera()->printSettings();

	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(0.125);

	// set a threshold. alternatively we could also activate automatic thresholding
	tracker->setThreshold(120);
	 // tracker->activateAutoThreshold(true);

	// let's use lookup-table undistortion for high-speed
	// note: LUT only works with images up to 1024x1024
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

	// switch to simple ID based markers
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	return 0;
}

void TrackerHelper::getMarkerPosition(TrackerMultiMarker* tracker, int index,
		Mat &pose) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(index);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);
	Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
	pose.at<float>(0, 0) = tmp.at<float>(3, 0);
	pose.at<float>(1, 0) = tmp.at<float>(3, 1);
	pose.at<float>(2, 0) = tmp.at<float>(3, 2);
}



void TrackerHelper::calRotYXZ(float pitch, float yaw, float roll, float Ys, float Zs, Mat &R) {
	// Todo: Notice the transformation.
	float yd = sin(pitch);
	float C = abs(tan(yaw));
	float zd = sqrt((1-yd*yd)/(C*C + 1));
	float xd = 0;
	// Careful recovery of direction from tan
	if ((yaw >= 0) & (yaw < CV_PI / 2)) {
		xd = C* yd;; // xd is the sin part.
	} else if ((yaw >= CV_PI / 2) & (yaw < CV_PI)) {
		xd = C*zd;
		zd *= -1;
	} else if ((yaw >= -CV_PI) & (yaw < -CV_PI / 2)) {
		xd = -C*zd;
		zd *= -1;
	} else if ((yaw >= -CV_PI / 2) & (yaw < 0)) {
		xd = -C* zd;
	}

	// cout << "Zs "<< Zs << endl;
	Mat Zm = (Mat_<float>(3, 1) << xd, yd, zd);
	Zm*= Zs;
	zd = Zm.at<float>(2,0);
	xd = Zm.at<float>(0,0);
	Mat X0 = (Mat_<float>(3, 1) << zd, 0, -xd);
	Mat Y0 = Zm.cross(X0);

	Mat Ym = Y0 * cos(roll) + (Y0.cross(Zm) * sin(roll))
			+ (Zm * Zm.dot(Y0) * (1 - cos(roll)));
	Ym *= Ys;
	// cout << "Cal Rot:" << endl<< "Zm" << Zm << endl<<"X0" << X0 << endl << "Y0" << Y0 <<
			// endl << "Ym" << Ym<< endl;
	Mat Xm = Ym.cross(Zm);
	R.at<float>(0, 0) = Xm.at<float>(0, 0);
	R.at<float>(1, 0) = Xm.at<float>(1, 0);
	R.at<float>(2, 0) = Xm.at<float>(2, 0);

	R.at<float>(0, 1) = Ym.at<float>(0, 0);
	R.at<float>(1, 1) = Ym.at<float>(1, 0);
	R.at<float>(2, 1) = Ym.at<float>(2, 0);

	R.at<float>(0, 2) = Zm.at<float>(0, 0);
	R.at<float>(1, 2) = Zm.at<float>(1, 0);
	R.at<float>(2, 2) = Zm.at<float>(2, 0);


}

void TrackerHelper::getAnglesYXZ(float &pitch, float &yaw, float &roll, float &Ys, float &Zs, Mat R) {

	Mat Zm;
	float yd = R.at<float>(1,2);

	normalize(R.col(2), Zm);

	yaw = atan2(Zm.at<float>(0, 0), Zm.at<float>(2, 0));
	pitch = asin(Zm.at<float>(1, 0));

//		cout << "Zm" << Zm << endl;
	Mat X0 = (Mat_<float>(3, 1) << Zm.at<float>(2, 0), 0, -Zm.at<float>(0, 0));
//		cout << "X0" << X0 << endl;
	Mat Ym = R.col(1);
	Mat Y0 = Zm.cross(X0);
//		cout << "Ym" << Ym << endl;
//		cout << "X0 dot Y" << Xm.dot(Y)<< endl;
//		cout << "norm Xm" << norm(Xm)<< endl;
//		cout << "Ym dot Y" << Ym.dot(Y)<< endl;
//		cout << "norm Ym" << norm(Ym)<< endl;
	roll = atan2(X0.dot(Ym) / norm(X0), Y0.dot(Ym) / norm(Y0));
	Ys = norm(Ym)/norm(Y0);
	Zs = yd/Zm.at<float>(1,0);
	// cout << "Get Angles:" << endl<< "Zm" << Zm << endl<< "X0" << X0 << endl << "Y0" << Y0 <<
				// endl << "Ym" << Ym<< endl;
}

void TrackerHelper::get6DOFMarkerPose(TrackerMultiMarker* tracker,	 Mat cam_pose,	Mat &marker_pose, int index) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(index);
	marker_pose = Mat::zeros(8,1, CV_32FC1);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);
	Mat T = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);

	// Construct a rotation matrix from global to camera frame.
	// First construct rotation matrix from global to camera frame
	Mat Rc2g;
	
	Rc2g.create(3, 3, CV_32FC1);

	calRotYXZ(cam_pose.at<float>(3, 0), cam_pose.at<float>(4, 0),
			cam_pose.at<float>(5, 0), cam_pose.at<float>(6, 0),
			cam_pose.at<float>(7, 0), Rc2g);

	Mat Trans_t;
	Trans_t.create(3, 4, CV_32FC1);
	hconcat(Rc2g, cam_pose.rowRange(0, 3), Trans_t);
	Mat Tc2g;
	 Tc2g.create(4, 4, CV_32FC1);
	Mat off = (Mat_<float>(1, 4) << 0, 0, 0, 1);
	vconcat(Trans_t, off,  Tc2g);

	// cout << "marker pose: " <<endl<<marker_pose<<endl;

	Mat Tm2c = T.t();
	Mat Tm2g = Tc2g * Tm2c;
	Mat Rm2g = Tm2g.rowRange(0, 3).colRange(0, 3);
	getAnglesYXZ(marker_pose.at<float>(3, 0), marker_pose.at<float>(4, 0),
				marker_pose.at<float>(5, 0), marker_pose.at<float>(6, 0),marker_pose.at<float>(7, 0),Rm2g);
	cout << "check12 " << endl;
	
	// Calculate Translation
	Mat m = Tm2g.col(3).rowRange(0,3);
	cout << "check13 " << endl;
	cout << "marker pose: " <<endl<<marker_pose<<endl;
	Mat mTrans = marker_pose.rowRange(0, 3);
	cout << "check14 " << endl;
	m.copyTo(mTrans);

	cout << "check15 " << endl;
	cout << "Tm2g" << endl<<Tm2g << endl;
//	cout << "trans" << endl << Tm2c << endl << endl << "T" << endl<< T << endl<< endl<< "Rm2c" << endl << Rm2c<< endl<< "Rm2c_t" << endl << Rm2c_t<< endl << endl;
	// cout <<"markerpose: " << endl << marker_pose << endl << endl;

}

// returns true if operation was a success. Else return false.
bool TrackerHelper::calcBigMarkerPose(TrackerMultiMarker* tracker,		Mat &marker_pose, Mat &T, bool firstCam) {
	static Mat acc = Mat::zeros(6,1, CV_32FC1);
	int markerWidth = 384;
	int dist = 930;
	Mat pose = Mat::zeros(6,1, CV_32FC1);
	bool result = false;
	for (int i = 0; i < 2; i++){
		ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(i);

		if ((markerInfo.id == 492 )||(markerInfo.id == 493)){
			ARFloat nOpenGLMatrix[16];
			ARFloat patternCentre[2] = { 0.0f, 0.0f };
			tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
					nOpenGLMatrix);
			Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
			tmp.copyTo(T);
			Mat Tm2c = T.t();
			Mat Rm2c = Tm2c.rowRange(0, 3).colRange(0, 3);

			getAnglesYXZ(marker_pose.at<float>(3, 0), marker_pose.at<float>(4, 0),
						marker_pose.at<float>(5, 0), marker_pose.at<float>(6, 0),marker_pose.at<float>(7, 0),Rm2c);
			Mat Rm2c_t = Mat::zeros(3,3, CV_32FC1);

			calRotYXZ(marker_pose.at<float>(3, 0), marker_pose.at<float>(4, 0),
					marker_pose.at<float>(5, 0), marker_pose.at<float>(6, 0),marker_pose.at<float>(7, 0) ,Rm2c_t);
			// Calculate Translation
			Mat m = Tm2c.col(3).rowRange(0,3);
			Mat mTrans = marker_pose.rowRange(0, 3);
			m.copyTo(mTrans);
			cout << "1. Marker pose"<< endl << marker_pose.rowRange(0, 6) << endl;

			if((markerInfo.id == 492 ) && (firstCam)) {
			// Get the center pose of the second marker.
				Mat distMat = (Mat_<float>(3,1) << dist, dist, dist);
				Mat temp = mTrans - distMat.mul(Rm2c.col(0));
				temp.copyTo(mTrans);
				result = true;
				cout << "2. Marker pose" << endl<< marker_pose.rowRange(0, 6) << endl;
				return result;
			} else if ((markerInfo.id == 493 ) && (!firstCam)) {
				return true;
			}
//			if (i == 0) {
//				pose = marker_pose.rowRange(0, 6);
////
//			}
//			if (i == 1) {
////
//				Mat temp = (pose - marker_pose.rowRange(0, 6));
//				if (markerInfo.id == 492) { //real
//					temp = -temp;
//				}
//				acc += temp;
//				count ++;
////				cout << "average  error: " << endl << acc/count << endl;
//			}
		}

	}
	return false;
}
// Get transformation matrix from camera to marker.
// Returns true if a new transformation matrix was obtained.
bool TrackerHelper::processMarkerImg(IplImage *img, TrackerMultiMarker *tracker,
		int width, int height, int id) {
	int numDetected = 0;
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
	cvCvtColor(img, tempImg, CV_RGB2GRAY);
	cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
			CV_THRESH_BINARY, 11);
	numDetected = tracker->calc((unsigned char*) greyImg->imageData);
	char name[10];
	sprintf(name, "%d", id);
//	cvShowImage(name, greyImg);
//	cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.
	cvReleaseImage(&greyImg);
	cvReleaseImage(&tempImg);
	if (numDetected != 0) {
//    printf("Number of Markers = %d\n\n", numDetected);

		return true;
	}
	return false;
}


int TrackerHelper::getNumDetected(IplImage *img, TrackerMultiMarker *tracker,
		int width, int height, char * name) {
	int numDetected = 0;
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
	cvCvtColor(img, tempImg, CV_RGB2GRAY);
	cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
			CV_THRESH_BINARY, 51);
	numDetected = tracker->calc((unsigned char*) greyImg->imageData);

	IplImage* new_img = cvCreateImage(cvSize(640, 480), greyImg->depth, greyImg->nChannels);
	cvResize(greyImg, new_img);

	cvShowImage(name, new_img);
	cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.
	
	cvReleaseImage(&new_img);
	cvReleaseImage(&greyImg);
	cvReleaseImage(&tempImg);
	return numDetected;
//
}

void TrackerHelper::getCameraPose(Mat* from_pose,
		Mat* to_pose, Mat* to_campose) {
	
	// Tg2m has to be reconstructed from marker orientation
	Mat Rm2g;
	Rm2g.create(3, 3, CV_32FC1);
	calRotYXZ(from_pose->at<float>(3, 0), from_pose->at<float>(4, 0),
			from_pose->at<float>(5, 0), from_pose->at<float>(6, 0), 
			from_pose->at<float>(7, 0), Rm2g);
	// cout << "check Zs" << from_pose->at<float>(7, 0) << endl;
	Mat off = (Mat_<float>(1, 4) << 0, 0, 0, 1);
	Mat Trans_t;
	Trans_t.create(3, 4, CV_32FC1);
	hconcat(Rm2g, from_pose->rowRange(0, 3), Trans_t);
	Mat Tm2g;
	Tm2g.create(4, 4, CV_32FC1);
	vconcat(Trans_t, off, Tm2g);

	 // cout << "Tm2g" << endl << Tm2g << endl << endl;
	 // cout << "From" << endl << *from_pose << endl << endl << "to_pose" << endl << *to_pose << endl << endl;
	// Tc2m has to be reconstructed from marker orientation
	Mat Rm2c;
	Rm2c.create(3, 3, CV_32FC1);
	calRotYXZ(to_pose->at<float>(3, 0), to_pose->at<float>(4, 0),
			to_pose->at<float>(5, 0), to_pose->at<float>(6, 0), 
			to_pose->at<float>(7, 0), Rm2c);
	hconcat(Rm2c, to_pose->rowRange(0, 3), Trans_t);
	Mat Tm2c;
	Tm2c.create(4, 4, CV_32FC1);
	vconcat(Trans_t, off, Tm2c);

	  // cout << "Tm2c" << endl << Tm2c << endl<< "Rm2c" << Rm2c << endl;
	 Mat Tc2m = Tm2c.inv();
	Mat Tc2g = Tm2g * Tc2m;
	// cout << "Tm2g" << endl << Tm2g << endl;
	Mat Rc2g = Tc2g.rowRange(0, 3).colRange(0, 3);
	getAnglesYXZ(to_campose->at<float>(3, 0), to_campose->at<float>(4, 0),
			to_campose->at<float>(5, 0), to_campose->at<float>(6, 0), 
			to_campose->at<float>(7, 0), Rc2g);
	Mat cam = Tc2g.col(3).rowRange(0, 3);
	Mat cam_trans = to_campose->rowRange(0, 3);
	cam.copyTo(cam_trans);
 // cout << "[Debug] cam_pose'= " << endl << *to_campose << endl << endl;

}


void TrackerHelper::getCameraPose(TrackerMultiMarker* tracker, Mat* cam_pose) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);

	Mat T = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);

	Mat Tc2m = T.t().inv();

	Mat Rc2m = Tc2m.rowRange(0, 3).colRange(0, 3);
	getAnglesYXZ(cam_pose->at<float>(3, 0), cam_pose->at<float>(4, 0),
			cam_pose->at<float>(5, 0),cam_pose->at<float>(6, 0),
		cam_pose->at<float>(7, 0), Rc2m);
	cout << endl << "Tc2m" << endl << Tc2m << endl;
	Mat cam = Tc2m.col(3).rowRange(0, 3);
	Mat cam_trans = cam_pose->rowRange(0, 3);
	cam.copyTo(cam_trans);
//  cout << "[Debug] cam_pose'= " << endl << cam_pose << endl << endl;

}
void TrackerHelper::saveCamPose(int id, Mat cam_pose) {
	// Here we save the id, device fullname and camera pose
	ofstream file;
	stringstream sbuffer;
	string filepath = "../main/cfg/campose";
	sbuffer << filepath << id;
	file.open(sbuffer.str().c_str(), ios::out | ios::trunc);
	if (!file.is_open()) {
		cout << "Error! Cannot open file named\" " << sbuffer.str()
				<< " \" to save Calibration" << endl;
		exit(EXIT_FAILURE);
	}
	/*
	 * Configuration data is of the form:
	 * ip:
	 * poseData:
	 */

//  cout << "Writing to file!" << endl;
	file << cam_pose.at<float>(0, 0) << "\n";  // x
	file << cam_pose.at<float>(1, 0) << "\n";  // y
	file << cam_pose.at<float>(2, 0) << "\n";  // z
	file << cam_pose.at<float>(3, 0) << "\n";  // roll
	file << cam_pose.at<float>(4, 0) << "\n";  // yaw
	file << cam_pose.at<float>(5, 0) << "\n";  // pitch
	file << cam_pose.at<float>(6, 0) << "\n";  // pitch
	file << cam_pose.at<float>(7, 0) << "\n";  // pitch
	file.close();
//  cout << "Written to file!" << endl;
}

void TrackerHelper::TransGraph::initialiseTrans() {
	rectangle(xcanvas, cvRect(0, 0, width, height), Scalar(255, 255, 255),
	CV_FILLED);
	rectangle(ycanvas, cvRect(0, 0, width, height), Scalar(255, 255, 255),
	CV_FILLED);
	rectangle(zcanvas, cvRect(0, 0, width, height), Scalar(255, 255, 255),
	CV_FILLED);
	line(xcanvas, Point(0, height / 2), Point(width, height / 2), 0, 4);
	line(ycanvas, Point(0, height / 2), Point(width, height / 2), 0, 4);
	line(zcanvas, Point(0, height), Point(width, height), 0, 4);

	int step = height / 20;
	for (int i = 0; i < height; i += step) {
		line(xcanvas, Point(0, i), Point(width, i), 0);
		line(ycanvas, Point(0, i), Point(width, i), 0);
		line(zcanvas, Point(0, i), Point(width, i), 0);
		char text[10];
		sprintf(text, "%d", height / 2 - i);
		putText(xcanvas, (const char*) text, Point(0, i), FONT_HERSHEY_PLAIN, 1,
				Scalar(0, 0, 255));
		putText(ycanvas, (const char*) text, Point(0, i), FONT_HERSHEY_PLAIN, 1,
				Scalar(0, 0, 255));
		sprintf(text, "%d", height - i);
		putText(zcanvas, (const char*) text, Point(0, i), FONT_HERSHEY_PLAIN, 1,
				Scalar(0, 0, 255));
	}

}

void TrackerHelper::TransGraph::drawTrans(Mat trans, const char* id) {
	/*
	 * The idea is that each time this function is called, the image is shifted by a proportional number of pixels
	 * to the left and then three separate 2D plots are generated of the trajectory for the x, y, z coordinates.
	 */
	// x-axis
	// Check for wrap around.
	pointer += 5;
	if (pointer >= width) {
		pointer = 0;
		initialiseTrans();
	}
	float coord1 = height / 2 - trans.at<float>(0, 0); // For x represent 40 cm in a whole.
	float coord2 = height / 2 - trans.at<float>(0, 1); // For x represent 40 cm in a whole.
	float coord3 = height - trans.at<float>(0, 2) / 2; // For x represent 40 cm in a whole.
	circle(xcanvas, Point(pointer, coord1), 1.0, Scalar(0, 255, 0));
	circle(ycanvas, Point(pointer, coord2), 1.0, Scalar(255, 0, 0));
	circle(zcanvas, Point(pointer, coord3), 1.0, Scalar(255, 0, 255));
	// Use currently increment pixel points by increment frame.
	// where we assume frames move with constant interval.

// Shift image to the left.
	stringstream title;
	title << "x" << id;
	imshow(title.str().c_str(), xcanvas);
	title.str("");
	title << "y" << id;
	imshow(title.str().c_str(), ycanvas);
	title.str("");
	title << "z" << id;
	imshow(title.str().c_str(), zcanvas);

	waitKey(1);
}

void TrackerHelper::RotGraph::initialiseDashBoard() {
	rectangle(dashBoard, cvRect(0, 0, dSize, dSize), Scalar(255, 255, 255),
	CV_FILLED);
//	circle(dashBoard, Point(dSize / 2, dSize / 2), 400, Scalar(255, 255, 255),
//			CV_FILLED);
	int step = 360 / 36;
	for (int i = -180; i <= 180; i += step) {
		line(dashBoard, Point((dSize / 2) + (2 * i), (dSize / 2) - 50),
				Point((dSize / 2) + (2 * i), (dSize / 2) + 50),
				Scalar(0, 0, 0)); // vertical line.
		char text[10];
		sprintf(text, "%d", i);
		putText(dashBoard, (const char*) text,
				Point((dSize / 2) + (2 * i) + 5, dSize / 2 + 5),
				FONT_HERSHEY_PLAIN, 0.5, Scalar(0, 0, 255));
		line(dashBoard, Point(dSize / 2 - 50, (dSize / 2) + (2 * i)),
				Point(dSize / 2 + 50, (dSize / 2) + (2 * i)), Scalar(0, 0, 0)); // horizontal line.
		putText(dashBoard, (const char*) text,
				Point(dSize / 2, (dSize / 2) - (2 * i) + 5), FONT_HERSHEY_PLAIN,
				0.5, Scalar(0, 0, 255));
	}
}

float TrackerHelper::RotGraph::radsToDegrees(float angle) {
	angle *= 180 / CV_PI;
	if (angle > 180) {
		angle -= 360;
	} else if (angle < -180) {
		angle += 360;
	}
	return angle;
}

void TrackerHelper::RotGraph::drawOrientation(Mat pose, const char* title) {
	initialiseDashBoard();
	// orientation is usually in radians. Need to transform to degrees.
	float pitch = 2 * radsToDegrees(pose.at<float>(0, 3));
	float yaw = 2 * radsToDegrees(pose.at<float>(0, 4));
	float roll = pose.at<float>(0, 5);

	Mat centrePoint;
	centrePoint.create(2, 1, CV_32FC1);

	// Account for pitch
	centrePoint.at<float>(0, 1) = dSize / 2 - pitch; // y-axis
	centrePoint.at<float>(0, 0) = dSize / 2; // x-axis

	// Account for yaw
	centrePoint.at<float>(0, 0) += yaw; // x-axis

	// Account for roll
	Mat T = (Mat_<float>(2, 2) << cos(roll), -sin(roll), sin(roll), cos(roll));
	Mat pointl = (Mat_<float>(2, 1) << -50, 0);
	Mat pointr = (Mat_<float>(2, 1) << 50, 0);
	pointl = T * pointl;
	pointr = T * pointr;

	// find middle point
	float x = centrePoint.at<float>(0, 0);
	float y = centrePoint.at<float>(0, 1);

	// draw circle at middle
	circle(dashBoard, Point(x, y), 3, Scalar(0, 255, 0));
	// Draw orientation line
	line(dashBoard,
			Point(pointl.at<float>(0, 0) + centrePoint.at<float>(0, 0),
					-pointl.at<float>(0, 1) + centrePoint.at<float>(0, 1)),
			Point(pointr.at<float>(0, 0) + centrePoint.at<float>(0, 0),
					-pointr.at<float>(0, 1) + centrePoint.at<float>(0, 1)),
			Scalar(0, 255, 0));
	imshow(title, dashBoard);
	waitKey(1);
}

void TrackerHelper::getCamPose(Mat * poseArray, int index) {
	ifstream file;
	stringstream sbuffer;
	string filepath = "../main/cfg/campose";
	sbuffer << filepath << index;
	file.open(sbuffer.str().c_str(), ios::in);
	if (!file.is_open()) {
		cout << "Error! Cannot open file to save Calibration" << endl;
		exit(EXIT_FAILURE);
	}
	char data[100];
	float x, y, z, roll, yaw, pitch, Ys, Zs;
	file.getline(data, 100);
	x = atof(data);
	file.getline(data, 100);
	y = atof(data);
	file.getline(data, 100);
	z = atof(data);
	file.getline(data, 100);
	roll = atof(data);
	file.getline(data, 100);
	yaw = atof(data);
	file.getline(data, 100);
	pitch = atof(data);
	file.getline(data, 100);
	Ys = atof(data);
	file.getline(data, 100);
	Zs = atof(data);
	file.close();
	*poseArray = (Mat_<float>(8, 1) << x, y, z, roll, yaw, pitch, Ys, Zs);
//	cout <<"[Debug] cam pose" << *poseArray << endl << endl;
}

void TrackerHelper::clusterData(Mat samples, Mat &centers) {
	int clusterCount = 5;
	Mat labels;
	int attempts = 5;
	kmeans(samples, clusterCount, labels,
			TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.01),
			attempts, KMEANS_PP_CENTERS, centers);
	cout << "Centers: " << endl << centers << endl;

}

void TrackerHelper::ImageReader::vidRead(Mat & img, timestamp_t &timestamp){
	cap.read(img);
	char data[200];
	timefile.getline(data, 200);
	stringstream instream;
	instream << data;
	instream >> timestamp;

	// timestamp = (float)((tempstamp>>3)%1000000);
	index++;
	cout << "INDEX: " << index << endl;
	cout << "timestamp: " << timestamp << endl;
}
void TrackerHelper::ImageReader::read(Mat & img){
	stringstream filename;
	filename << dirname.str() << "/pic"<<index<<".jpg";
	img = imread(filename.str().c_str(), CV_LOAD_IMAGE_COLOR );
	cout << "Image path: "<< filename.str() << endl;
	index++;
}
