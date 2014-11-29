/*
 * helper.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: parallels
 */
#include "trackerhelper.h"

// The pose of the initial camera.

int markerWidth = 126;
void TrackerHelper::initialiseCapture(int id, VideoCapture &cap) {
	// Write a function to open camera
	// we will use an OpenCV capture
	string file =
			"/media/psf/Home/Documents/PhdBase/Main/Helper_Projects/More_T2/video/video_cam_";
	stringstream buffer;
	// Todo: Change to right format.
	buffer << file << id << ".mov";
	cap.open(buffer.str().c_str());
//	cap.open(id);
//	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
//	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	printf("Opening calibration video number : %d", id);
	if (!cap.isOpened()) {
		std::cout << "Could not initialize capturing...\n" << std::endl;
		exit(EXIT_FAILURE);
	}
}

void TrackerHelper::initFrameSize(VideoCapture &cap, int &width, int &height) {
	// Write a function to open camera
	Mat frame;
	IplImage *img;
	cap.read(frame);
	img = cvCreateImage(cvSize(frame.cols, frame.rows), IPL_DEPTH_8U,
			frame.channels());
	img->imageData = (char *) frame.data;
	if (frame.empty()) {
		printf("Failed to open image from capture device");
		exit(EXIT_FAILURE);
	}
	width = img->width;
	height = img->height;
}

int TrackerHelper::configTracker(int width, int height) {
	// write a function get camera frame size.
	tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

	// load a camera file.
	if (!tracker->init(
			"/home/parallels/catkin_ws/src/more_t2/data/cam0/all.cal",
			"/home/parallels/tools/ARToolKitPlus-2.3.1/sample/data/markerboard_480-499.cfg",
			1.0f, 1000.0f)) // load MATLAB file
			{
		ROS_INFO("ERROR: init() failed\n");
		exit(EXIT_FAILURE);
	}

	tracker->getCamera()->printSettings();

	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(0.125);

	// set a threshold. alternatively we could also activate automatic thresholding
	tracker->setThreshold(160);

	// let's use lookup-table undistortion for high-speed
	// note: LUT only works with images up to 1024x1024
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

	// switch to simple ID based markers
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	return 0;
}
int TrackerHelper::configTracker(int width, int height, const char* cal) {
	// write a function get camera frame size.
	tracker = new TrackerMultiMarker(width, height, 8, 6, 6, 6, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

	// load a camera file.
	if (!tracker->init(cal,
			"/home/parallels/tools/ARToolKitPlus-2.3.1/sample/data/markerboard_480-499.cfg",
			1.0f, 1000.0f)) // load MATLAB file
			{
		ROS_INFO("ERROR: init() failed\n");
		exit(EXIT_FAILURE);
	}

	tracker->getCamera()->printSettings();

	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(0.125);

	// set a threshold. alternatively we could also activate automatic thresholding
	tracker->setThreshold(160);

	// let's use lookup-table undistortion for high-speed
	// note: LUT only works with images up to 1024x1024
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

	// switch to simple ID based markers
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	return 0;
}
void TrackerHelper::getRotMatrix(float roll, float yaw, float pitch, Mat &R) {
	float cosA = cos(roll), sinA = sin(roll), cosB = cos(yaw), sinB = sin(yaw),
			cosC = cos(pitch), sinC = sin(pitch);
	Mat Rz = (Mat_<float>(3, 3) << cosC, -sinC, 0, sinC, cosC, 0, 0, 0, 1);
	Mat Rx = (Mat_<float>(3, 3) << 1, 0, 0, 0, cosA, -sinA, 0, sinA, cosA);
	Mat Ry = (Mat_<float>(3, 3) << cosB, 0, sinB, 0, 1, 0, -sinB, 0, cosB);
	Mat temp = Rz * Rx * Ry;
	// Todo: Notice the transformation.
	((Mat) temp.t()).copyTo(R);
}

void TrackerHelper::getMarkerPose(TrackerMultiMarker* tracker, int index,
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

void TrackerHelper::getMarkerT(TrackerMultiMarker* tracker, int index, Mat &Tm,
		Mat Tc) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(index);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);
	Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
	Mat(Tc * tmp.t()).copyTo(Tm);
//					cout << "getMarker Tc2m:" << endl << tmp.t() << endl <<"Marker Tm" << endl << Tm << endl;
}
void TrackerHelper::getCameraT(TrackerMultiMarker* tracker, int index, Mat &Tc,
		Mat Tm) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(index);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);
	Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
	Mat(Tm * Mat(tmp.t()).inv(CV_LU)).copyTo(Tc);
//					cout <<  "getCamera Tc2m:" << endl << Mat(tmp.t()).inv()<< endl <<"Camera Tm" << endl << Tm << endl << "Tc" <<endl << Tc<< endl;
}

// Todo added -ve sign to sin roll in rodrigues's formula.
// Todo The function produces inaccuracies in rotation.
void TrackerHelper::calRotYXZ(float pitch, float yaw, float roll, Mat &R) {
	// Todo: Notice the transformation.
	double xd, zd;
	// Careful recovery of direction from tan
	if ((yaw >= 0) & (yaw < CV_PI / 2)) {
		xd = abs(tan(yaw)); // xd is the sin part.
		zd = 1;
	} else if ((yaw >= CV_PI / 2) & (yaw < CV_PI)) {
		xd = abs(tan(yaw));
		zd = -1;
	} else if ((yaw >= -CV_PI) & (yaw < -CV_PI / 2)) {
		xd = -abs(tan(yaw));
		zd = -1;
	} else if ((yaw >= -CV_PI / 2) & (yaw < 0)) {
		xd = -abs(tan(yaw));
		zd = 1;
	}

	double yd = sin(pitch);

	Mat Zm = (Mat_<double>(3, 1) << xd, yd, zd);
	Mat X0 = (Mat_<double>(3, 1) << zd, 0, -xd);
	Mat Y0 = Zm.cross(X0);
	Mat zm, ym, xm;
	normalize(Zm, zm);
	Mat Ym = Y0 * cos(roll) + zm.cross(Y0) * sin(-roll)
			+ (zm * zm.dot(Y0) * (1 - cos(roll)));

	normalize(Ym, ym);
	xm = ym.cross(zm);
//	cout << "norm xm" << norm(xm) << endl;
	R.at<float>(0, 0) = xm.at<double>(0, 0);
	R.at<float>(1, 0) = xm.at<double>(1, 0);
	R.at<float>(2, 0) = xm.at<double>(2, 0);

	R.at<float>(0, 1) = ym.at<double>(0, 0);
	R.at<float>(1, 1) = ym.at<double>(1, 0);
	R.at<float>(2, 1) = ym.at<double>(2, 0);

	R.at<float>(0, 2) = zm.at<double>(0, 0);
	R.at<float>(1, 2) = zm.at<double>(1, 0);
	R.at<float>(2, 2) = zm.at<double>(2, 0);
//	Mat R0x = R.col(0);
//	xm.copyTo(R0x);
//	Mat R1x = R.col(1);
//	ym.copyTo(R1x);
//	Mat R2x = R.col(2);
//	zm.copyTo(R2x);

}
void TrackerHelper::getAnglesYXZ(float &pitch, float &yaw, float &roll, Mat R) {

	Mat Zm;
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

}
void TrackerHelper::calcMarkerPose(TrackerMultiMarker* tracker, Mat cam_pose,
		Mat &marker_pose, Mat &T) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);
	Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
	tmp.copyTo(T);

	// Calculate orientation.

	// Construct a rotation matrix from global to camera frame.
	// First construct rotation matrix from global to camera frame
	Mat Rc2g;
	Rc2g.create(3, 3, CV_32FC1);
	calRotYXZ(cam_pose.at<float>(3, 0), cam_pose.at<float>(4, 0),
			cam_pose.at<float>(5, 0), Rc2g);
	Mat Rm2c = T.rowRange(0, 3).colRange(0, 3).t();
	Mat Rm2g = Rc2g * Rm2c;
	getAnglesYXZ(marker_pose.at<float>(3, 0), marker_pose.at<float>(4, 0),
			marker_pose.at<float>(5, 0), Rm2g);
//  marker.mTime = ros::Time::now();

	// test rotation
	Mat Rm2g_t = Mat(3, 3, CV_32FC1);
	calRotYXZ(marker_pose.at<float>(3, 0), marker_pose.at<float>(4, 0),
			marker_pose.at<float>(5, 0), Rm2g_t);
	cout << "Start!" << endl << "Rm2g Diff" << endl << Rm2g -Rm2g_t
			<< endl << endl;

	Mat cPose = (Mat_<float>(4, 1) << 0, 0, 0, 1);
	Mat marker_t = ((Mat) (T.t() * cPose));

	Mat Trans_t;
	Trans_t.create(3, 4, CV_32FC1);
	hconcat(Rc2g, cam_pose.rowRange(0, 3), Trans_t);

	Mat Trans;
	Trans.create(4, 4, CV_32FC1);
	Mat off = (Mat_<float>(1, 4) << 0, 0, 0, 1);
	vconcat(Trans_t, off, Trans);
//	cout << "cam trans" << endl << cam_pose.rowRange(0, 3) << endl <<endl;
//	cout <<"Start!"<<endl<<"Rc2g" << Rc2g <<endl<<"Rm2c"<<Rm2c<<endl << endl;
	// Calculate Translation
	Mat m = ((Mat) (Trans * marker_t)).rowRange(0, 3);
	Mat mTrans = marker_pose.rowRange(0, 3);
	m.copyTo(mTrans);
//	cout <<"markerpose: " << endl << marker_pose << endl << endl;
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
			CV_THRESH_BINARY, 111);
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
		int width, int height, int id) {
	int numDetected = 0;
	IplImage *greyImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	IplImage *tempImg = cvCreateImage(cvSize(width, height), img->depth, 1);
	cvCvtColor(img, tempImg, CV_RGB2GRAY);
	cvAdaptiveThreshold(tempImg, greyImg, 255.0, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
			CV_THRESH_BINARY, 111);
	numDetected = tracker->calc((unsigned char*) greyImg->imageData);
	char name[10];
	sprintf(name, "%d", id);
	cvShowImage(name, greyImg);
	cvWaitKey(1); // Wait for image to be rendered on screen. If not included, no image is shown.
	cvReleaseImage(&greyImg);
	cvReleaseImage(&tempImg);
	return numDetected;
//
}

void TrackerHelper::getCameraPose(TrackerMultiMarker* tracker,
		Mat* marker_pose_t, Mat* cam_pose, Mat&T) {
	ARToolKitPlus::ARMarkerInfo markerInfo = tracker->getDetectedMarker(0);
	ARFloat nOpenGLMatrix[16];
	ARFloat patternCentre[2] = { 0.0f, 0.0f };
	tracker->calcOpenGLMatrixFromMarker(&markerInfo, patternCentre, markerWidth,
			nOpenGLMatrix);

	Mat tmp = Mat(4, 4, CV_32FC1, (float *) nOpenGLMatrix);
	tmp.copyTo(T);
//  cout << "[Debug]: T' = " << endl << T.t() << endl << endl;
	Mat Tc2m = ((Mat) (T.t().inv() * centreMat)).rowRange(0, 3);

//  cout << "[Debug]: temp' = " << endl << temp << endl << endl;
	Mat cam_trans = cam_pose->rowRange(0, 3);

//  cout << "[Debug]: cam_trans' = " << endl << cam_trans << endl << endl;
	// Calculate cam's orientation pose.

//  cout << "[Debug]: Rc2m' = " << endl << Rc2m << endl << endl;
	// Rg2m has to be reconstructed from marker orientation
	Mat Rc2m = T.rowRange(0, 3).colRange(0, 3);
	Mat Rm2g;
	Rm2g.create(3, 3, CV_32FC1);
	calRotYXZ(marker_pose_t->at<float>(3, 0), marker_pose_t->at<float>(4, 0),
			marker_pose_t->at<float>(5, 0), Rm2g);
	Mat Rc2g = Rm2g * Rc2m;
	getAnglesYXZ(cam_pose->at<float>(3, 0), cam_pose->at<float>(4, 0),
			cam_pose->at<float>(5, 0), Rc2g);

	// Translation
	// first get the camera's position in terms of the marker coordinates
	// Reconstruct transformation matrix from world to marker
	Mat mPose = (Mat_<float>(4, 1) << 0, 0, 0, 1);
	Mat cam_t = ((Mat) (T.t().inv(CV_SVD) * mPose));
	Mat off = (Mat_<float>(1, 4) << 0, 0, 0, 1);
	//Calculate T inv

	// Then transforms from the marker frame to the global frame.
	// Frame reconstruct the transformation matrix

	Mat Trans_t;
	Trans_t.create(3, 4, CV_32FC1);
	hconcat(Rm2g, marker_pose_t->rowRange(0, 3), Trans_t);
	Mat Trans;
	Trans.create(4, 4, CV_32FC1);
	vconcat(Trans_t, off, Trans);
	//  cout << "trans" << endl << Trans << endl << endl << "marker pose" << endl<< *marker_pose_t << endl<< endl<< "Rg2m" << endl << Rg2m<< endl << endl;

	Mat cam = ((Mat) (Trans * cam_t)).rowRange(0, 3);
	cam.copyTo(cam_trans);
//  cout << "[Debug] cam_pose'= " << endl << cam_pose << endl << endl;

}
void TrackerHelper::writeCalibrationToFile(int id, Mat cam_pose) {
	// Here we save the id, device fullname and camera pose
	ofstream file;
	stringstream sbuffer;
	string filepath = "/home/parallels/catkin_ws/src/more_t2/cfg/cfg_";
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
	string filepath = "/home/parallels/catkin_ws/src/more_t2/cfg/cfg_";
	sbuffer << filepath << index;
	file.open(sbuffer.str().c_str(), ios::in);
	if (!file.is_open()) {
		cout << "Error! Cannot open file to save Calibration" << endl;
		exit(EXIT_FAILURE);
	}
	char data[100];
	float x, y, z, roll, yaw, pitch;
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
	file.close();
	*poseArray = (Mat_<float>(6, 1) << x, y, z, roll, yaw, pitch);
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
