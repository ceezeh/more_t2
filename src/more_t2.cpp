#include "trackerhelper.h"
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;

Mat cam_pose;
TrackerHelper helper;


int main(int argc, char** argv)
{

  ros::init(argc, argv, "more_t2");

  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);

  int noCams = 2;
  n.getParam("noCams", noCams); //The total number of possible cameras.
  // Generate cam_pose array based on number of cameras and referenced by ID;

  // Generate tracking information for all relevant camera feeds.
  for (int id = 1; id < noCams; id++)
  {

    // Initialise capturing device.
    // We assume frames are all of the same size and are all gray scale.
    stringstream sbuffer;
    string filepath = "/media/psf/Home/Documents/PhdBase/Main/Helper_Projects/More_T2/video/video_cam_";
    sbuffer << filepath << id << ".mov";

    int width, height;
   VideoCapture capture;
   helper.initialiseCapture(id, capture);
    //Initiliase width and heigth.
    helper.initFrameSize(capture, width, height);

    // Get the camera's pose

    helper.getCamPose(&cam_pose, id);

    if (helper.configTracker(width, height) != 0)
    {
      cout << "Error! Cannot configure tracker." << endl;
      exit(EXIT_FAILURE);
    }

    Mat pose;
    pose.create(6, 1, CV_32F);
    Mat T;
    T.create(4, 4, CV_8UC(2));


    float t = 0; // Todo: Correct for time variable.

    string path = "/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_";
    sbuffer << path << id << ".csv";
    cout << "sbuffer" << endl << sbuffer.str() << endl;
    cout << "id" << endl << id << endl;
    // File stream to save data.
    ofstream fp;
    fp.open(sbuffer.str().c_str(), ios::out | ios::trunc);
    sbuffer.str("");
    if (!fp.is_open())
    {
      cout << "Error! Cannot open file to save pose data" << endl;
      exit(EXIT_FAILURE);
    }
    Mat tempImg;
    capture.read(tempImg);
    if (tempImg.empty()) {
        		printf("End of File!");
        		        break;
        	}
    IplImage* img = cvCreateImage(cvSize(tempImg.cols, tempImg.rows), IPL_DEPTH_8U,
			tempImg.channels());

    // Todo: While video is not empty
    while (ros::ok())
    {
    	capture.read(tempImg);
    	if (tempImg.empty()) {
    		printf("End of File!");
    		        break;
    	}
    bool newResult = helper.processMarkerImg(img, helper.tracker, width, height, id);
      if (newResult)
      {
        helper.calcMarkerPose(helper.tracker,cam_pose, pose, T);
        // Todo: Implement Constant frame rate!
        // Save pose to file with  time stamp.
        // To do this, increment each time based on frame rate.
        // Here we assume frame rate is constant as given by a hardware trigger.

        // We save each row in the form, x, y, z, t;
        // Todo: Save information on 6DOF trajectory so include:
        // Yaw, Pitch and Row information.
        fp << pose.at<float>(0, 0) << "," << pose.at<float>(1, 0) << "," << pose.at<float>(2, 0) << ","
            << pose.at<float>(3, 0) << "," << pose.at<float>(4, 0) << "," << pose.at<float>(5, 0) << "," << t << "\n";
      }
      // Todo: Scale time by frame rate factor.
      t++;

      ros::spinOnce();
      loop_rate.sleep();
    }
    capture.release();
    fp.close();
  }

  return 0;
}
