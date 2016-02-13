#include "trackerhelper.h"
#include <errno.h>      // Error number definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
using namespace std;
using namespace cv;


int fd;
void fnExit1 (void)
{
	close(fd);;
}
int main(int argc, char** argv) {
	atexit (fnExit1);
	

	// Generate cam_pose array based on number of cameras and referenced by ID;

	// Generate tracking information for all relevant camera feeds.
	if (argc != 2) {
		cout << "Invalid number of arguments" << endl;
		exit(1);
	}

	
	char* myfifo = "/tmp/triggergen";
    /* create the FIFO (named pipe) */
	mkfifo(myfifo, 0666);
	fd = open(myfifo, O_WRONLY);

	string dirname = argv[1];

	TrackerHelper::ImageReader imReader(dirname);

	timestamp_t timestamp;
	Mat dst;
	Size size(640,480);
	Mat src;//src image
	bool ffwd = false;
	// Todo: While video is not empty 
	while (1) {
		imReader.vidRead(src, timestamp);
		if (ffwd){
			imReader.vidRead(src, timestamp);
			imReader.vidRead(src, timestamp);
			imReader.vidRead(src, timestamp);
			imReader.vidRead(src, timestamp);
		}
		if (src.empty()) {
			printf("End of File!\n");
			exit(1);
		}

	resize(src,dst,size);//resize image
	imshow("Hi",dst);

	cvWaitKey(30); // Wait for image to be rendered on screen. If not included, no image is shown.
	fd_set readset;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 100;

	FD_ZERO(&readset);
	FD_SET(fileno(stdin), &readset);

	select(fileno(stdin)+1, &readset, NULL, NULL, &tv);
        // the user typed a character so exit
	if(FD_ISSET(fileno(stdin), &readset))
	{
		char c = fgetc (stdin);
		if (c == 'p') { // pause viewing
			while (1) {
				imshow("Hi", dst);
				cvWaitKey(30);
				// Listen for input
				FD_ZERO(&readset);
				FD_SET(fileno(stdin), &readset);

				select(fileno(stdin)+1, &readset, NULL, NULL, &tv);
			        // the user typed a character so exit
				if(FD_ISSET(fileno(stdin), &readset))
				{
					char c = fgetc (stdin);
					if (c == 'p') break;
					else if (c =='c') write(fd, &timestamp, sizeof(unsigned long long));
					else if (c == 'f') ffwd = !ffwd;
				}
			}

		}
		else if (c =='c') write(fd, &timestamp, sizeof(unsigned long long));
		else if (c == 'f') ffwd = !ffwd;
	}
	src.release();
	dst.release();
}
close(fd);
return 0;
}


