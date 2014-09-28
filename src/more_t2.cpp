// Include the ROS C++ APIs
#include <ros/ros.h>
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <cstdio>
#include <ARToolKitPlus/TrackerSingleMarker.h>
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerSingleMarker;


int artest(unsigned char* cameraBuffer, const int width, const int height);


// Standard C++ entry point





int main(int argc, char** argv) {
	
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

  	ros::Rate loop_rate(10);

    VideoCapture cap("/home/parallels/catkin_ws/src/more_t2/videos/test1.mov"); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    const int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    const int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    cout << "Frame per seconds : " << fps << endl;

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while (ros::ok())
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
                        cout << "Cannot read the frame from video file" << endl;
                       break;
        }

        imshow("MyVideo", frame); //show the frame in "MyVideo" window
        imdecode(frame, CV_LOAD_IMAGE_GRAYSCALE, &frame);
        artest(frame.data, width, height);
        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
       	{
                cout << "esc key is pressed by user" << endl; 
                break; 
       	}

       	ROS_INFO("Check!");
       	ros::spinOnce();

    	loop_rate.sleep();

    }

    return 0;
}


int artest(unsigned char* cameraBuffer, const int width, const int height) {
    // switch this between true and false to test
    // simple-id versus BCH-id markers
    const bool useBCH = false;

    const int bpp = 1;
    size_t numPixels = width * height * bpp;
    /*size_t numBytesRead;
    const char *fName = useBCH ? "/home/parallels/catkin_ws/src/more_t2/data/image_320_240_8_marker_id_bch_nr0100.raw"
            : "/home/parallels/catkin_ws/src/more_t2/data/image_320_240_8_marker_id_simple_nr031.raw";

    unsigned char cameraBuffer[numPixels];

    // try to load a test camera image.
    // these images files are expected to be simple 8-bit raw pixel
    // data without any header. the images are expetected to have a
    // size of 320x240.
    if (FILE* fp = fopen(fName, "rb")) {
        numBytesRead = fread(cameraBuffer, 1, numPixels, fp);
        fclose(fp);
    } else {
        ROS_INFO("Failed to open %s\n", fName);
        return -1;
    }

    if (numBytesRead != numPixels) {
        ROS_INFO("Failed to read %s\n", fName);
        return -1;
    }
*/
    // create a tracker that does:
    //  - 6x6 sized marker images (required for binary markers)
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 0 non-binary pattern
    //  - can detect a maximum of 8 patterns in one imagege
    TrackerSingleMarker tracker(width, height, 8, 6, 6, 6, 0);

    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    //tracker.setLoadUndistLUT(true);

    // load a camera file.
    if (!tracker.init("/home/parallels/catkin_ws/src/more_t2/data/PGR_M12x0.5_2.5mm.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        ROS_INFO("ERROR: init() failed\n");
        return -1;
    }

    tracker.getCamera()->printSettings();

    // define size of the marker in OpenGL units
    tracker.setPatternWidth(2.0);

    // the marker in the BCH test image has a thin border...
    tracker.setBorderWidth(useBCH ? 0.125 : 0.25);

    // set a threshold. alternatively we could also activate automatic thresholding
    tracker.setThreshold(150);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker.setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

    // do the OpenGL camera setup
    // glMatrixMode(GL_PROJECTION)
    // glLoadMatrixf(tracker.getProjectionMatrix());

    // here we go, just two calls to find the camera pose
    vector<int> markerId = tracker.calc(cameraBuffer);

    tracker.selectBestMarkerByCf();
    float conf = tracker.getConfidence();

    // use the result of calc() to setup the OpenGL transformation
    // glMatrixMode(GL_MODELVIEW)
    // glLoadMatrixf(tracker.getModelViewMatrix());
    // printf("\n\nFound marker  (confidence %d)\n\nPose-Matrix:\n  ", (int(conf * 100.0f)));
    for (int i = 0; i < 16; i++)
        printf("%.2f  %s", tracker.getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
    return 0;

}