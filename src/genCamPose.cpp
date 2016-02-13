#include "trackerhelper.h"
#include <vector>
#include <numeric>      // std::accumulate
#define LIMIT1 10000	// for position
#define LIMIT2 2*CV_PI // for orientation
#define LIMIT3 1.5
using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerMultiMarker;


TrackerHelper helper;
int max_samples = 500;
ifstream fromFile, toFile;



#define NBUCKET 500 /* bucket size */

typedef std::vector<float> Bucket; 


float BucketSort(std::vector<float>arr, float limit);
void printBuckets(std::vector<float> v);
int getBucketIndex(float value, float limit);

float BucketSort(std::vector<float>arr,float limit)
{	
	Bucket buckets[NBUCKET];  

	/* allocate memory for array of pointers to the buckets */
	// buckets = (struct Node **)malloc(sizeof(struct Node*) * NBUCKET);

	for(int i = 0; i< NBUCKET; i++) {
		buckets[i] = std::vector<float> ();
	}
	
	// cout << "size: " << arr.size()<< endl;
	/* put items into the buckets */
	for(int i = 0; i < arr.size(); i++) {
		int pos = getBucketIndex(arr[i],limit);
		buckets[pos].push_back(arr[i]);
	}
	

	/* check what's in each bucket */
	for(int i = 0; i < NBUCKET; i++) {
		cout << "Bucket[" << i << "] : size: ["<< buckets[i].size()<<" ] : ";
		printBuckets(buckets[i]);
		cout << endl;
	}

	int index = -1;
	int max = -1;
	// find bin with most elements
	for (int i = 0; i < NBUCKET; i++) {
		
		int temp = buckets[i].size();
		if (temp> max){
			max = temp;
			index = i;
		}
	}
	// Get mean of bucket with most elements
	float sum = std::accumulate(buckets[index].begin(), buckets[index].end(), 0.0);
	float mean = sum / buckets[index].size();


	/* free memory */
	for(int i = 0; i < NBUCKET;++i) {	
		buckets[i].clear();
	}
	return mean;
}

int getBucketIndex(float value, float limit)
{
	float interval = 2*limit/NBUCKET;
	float temp = value/ float(interval);
	return (float(NBUCKET)/2) +temp ;
}

void printBuckets(std::vector<float> bucket)
{
	for (int i = 0; i < bucket.size(); i++) {
		cout << setw(3) << bucket[i];
	}
}
void getline_t(ifstream * in, Mat *result) {
	char res[1000];
	in->getline(res, 1000);

	float fromx, fromy, fromz, fromx1, fromy1, fromz1, fromYs, fromZs;
	char c1, c2, c3, c4, c5, c6, c7; //to eat the commas
	stringstream sbuffer(res);
	sbuffer >> fromx >> c1 >> fromy>> c2 >> fromz >> c3 >> fromx1 >> c4>> fromy1 >> c5>> fromz1 >> c6 >> fromYs >> c7 >> fromZs;
	*result = (Mat_<float>(8,1) << fromx, fromy, fromz, fromx1, fromy1, fromz1, fromYs, fromZs);
}

int rejectOutlier (std::vector<float> *acc, float *result) {
	// calculate summation
	float sum[11];
	for (int i = 0; i < 11; i++) {
		sum[i] = 0;
	}
	int size = acc[0].size();
	for (int i = 0; i < size; i++){
		for (int j = 0; j < 8; j++) {
			if (j < 3) {
				sum [j] += acc[j].at(i);
			}
			if ((j >= 3)&&(j <= 5)) {
				int index = 2*(j-3) + 3;

				sum [index] += sin(acc[j].at(i));
				sum [index+1] += cos(acc[j].at(i));
			}
			if (j > 5) {
				sum [j+3] += acc[j].at(i);
			}
		}
	}

	// calculate average
	for (int i = 0; i < 11 ; i++) {
		sum[i] /= size;
	}
	float average[8] = {sum [0], sum[1], sum[2], atan2(sum[3], sum[4]), atan2(sum[5], sum[6]),
		atan2(sum[7], sum[8]), sum[9], sum[10]}; 


	// Detect outlier and move it to the end of the list to be ignored
		int endCap = 0;

		for (int i = 0; i < size; i++){
			for (int j = 0; j < 8; j++) {
				cout << "endcap " << endCap << endl; 
				if ((j < 3)||(j >5)) {
					float diff = abs(acc[j].at(i) - average[j]);

					if ( diff > 100) {
						for (int k = 0; k < 8; k++) {
							float temp = acc[k].at(size - 1 - endCap);
							acc[k].at(size - 1 - endCap) = acc[k].at(i);
							acc[k].at(i) = temp;

						}
						endCap++;
						if (endCap == max_samples)return -1;
						continue;
					} 
				}
			// cout << "check!" << endl;
				if ((j > 2)&&(j<6)) {
				//convert different to acute angle.
					float diff = abs(acc[j].at(i) - average[j]);
					if (diff > CV_PI) {
						diff = 2*CV_PI - diff;
					}
				// // if (diff > 1.2) {
				// 		cout << "diff " << diff << endl;
				// cout << "ave " << average[j] << endl;
				// cout << "acc " << acc[j].at(i) << endl;
					if (diff > 0.5) {
						for (int k = 0; k < 8; k++) {
							float temp = acc[k].at(size - 1 - endCap);
							acc[k].at(size - 1 - endCap) = acc[k].at(i);
							acc[k].at(i) = temp;

						}
						endCap++;
						if (endCap == max_samples)return -1;
						continue;
					}
				}
			}
		}

	// recalculate average 
		for (int i = 0; i < 11; i++) {
			sum[i] = 0;
		}
		size = acc[0].size() - endCap;
		cout << "size: " <<  size <<endl;
		for (int i = 0; i < size; i++){
			for (int j = 0; j < 8; j++) {
				if (j < 3) {
					sum [j] += acc[j].at(i);
				}
				if ((j >= 3)&&(j <= 5)) {
					int index = 2*(j-3) + 3;

					sum [index] += sin(acc[j].at(i));
					sum [index+1] += cos(acc[j].at(i));
				}
				if (j > 5) {
					sum [j+3] += acc[j].at(i);
				}
			}
		}

		if (size > 0) {
		// calculate average
			for (int i = 0; i < 11 ; i++) {
				sum[i] /=size;
			}
			result[0] = sum[0];
			result[1] = sum[1];
			result[2] = sum[2];
			result[3] = atan2(sum[3], sum[4]);
			result[4] = atan2(sum[5], sum[6]);
			result[5] = atan2(sum[7], sum[8]);
			result[6] = sum[9];
			result[7] = sum[10]; 
			return 0;
		} else {
			result[0] = result[1] = result[2] = result[3] = result[4] = result[5] = result[6] = result[7] = -1; 
			return -1;
		}

	}


	void fnExit1 (void)
	{
		fromFile.close();
		toFile.close();
	}

	void saveCamPose(Mat cam_pose, const char* filename) {
		ofstream file;
		file.open(filename, ios::out | ios::trunc);
		if (!file.is_open()) {
			cout << "Error! Cannot open file named\" " << filename
			<< " \" to save Calibration" << endl;
			exit(EXIT_FAILURE);
		}
	file << cam_pose.at<float>(0, 0) << "\n";  // x
	file << cam_pose.at<float>(1, 0) << "\n";  // y
	file << cam_pose.at<float>(2, 0) << "\n";  // z
	file << cam_pose.at<float>(3, 0) << "\n";  // roll
	file << cam_pose.at<float>(4, 0) << "\n";  // yaw
	file << cam_pose.at<float>(5, 0) << "\n";  // pitch
	file << cam_pose.at<float>(6, 0) << "\n";  // pitch
	file << cam_pose.at<float>(7, 0) << "\n";  // pitch
	file.close();


}

int main(int argc, char** argv) {
	atexit (fnExit1);

	string path;
	char opt;
	string fromCamName;
	string toCamName;

	while ((opt = getopt(argc, argv, "f:l:t:")) != -1) {
		switch (opt) {
			case 'f': // from camera name
			{
				fromCamName = optarg;
				cout << "From camera path: "<< optarg <<endl;
				break;
			}
			case 'l':
			{
				path = optarg;
				cout << "data path : "<<path<<endl;
				break;
			}
			case 't':
			{
				toCamName = optarg;
				
				cout << "to camera path: "<< optarg << endl;
				break;
			}
		}
	}



	Mat toCampose = (Mat_<float>(8,1) << 0,0,0,0,0,0,1,1);

	if (!fromCamName.empty()){
		string fromCamPath = path + "/"+fromCamName +"/pose.csv";
		cout << " From campath: "<<fromCamPath << endl;
		fromFile.open(fromCamPath.c_str(), ios::in); 
		if (!fromFile.is_open()) {
			std::cout << "Failed to open from cam path file\n" << std::endl;
			exit(EXIT_FAILURE);
		}

	}
	

	string toCamPath = path + "/"+toCamName +"/pose.csv";
	toFile.open(toCamPath.c_str(), ios::in); 
	cout << " To campath: "<<toCamPath << endl;
	if (!toFile.is_open()) {
		std::cout << "Failed to open to cam path file\n" << std::endl;
		exit(EXIT_FAILURE);
	}


	int count = 0;
	std::vector<float> acc[11];

	Mat accPosition = Mat::zeros(3, 1, CV_32FC1);
	Mat fromMat = (Mat_<float>(8,1) << 0, 0, 0, 0, 0, 0, 1, 1);
	Mat toMat = Mat::zeros(8, 1, CV_32FC1);
	
	// Skip first header line.
	char indata[500];
	if (!fromCamName.empty()){
		fromFile.getline(indata,500);
	}
	toFile.getline(indata,500);

	// Open file to save campose data
	string resultPath = path + "/pose.csv";
	cout << "results path: " <<  resultPath << endl;
	ofstream fp;
	fp.open(resultPath.c_str(), ios::out | ios::trunc);
	if (!fp.is_open()) {
		cout << "Error! Cannot open file to save result data" << endl;
		exit(EXIT_FAILURE);
	}


	while (true) {
		if (!fromCamName.empty()){
			getline_t(&fromFile, &fromMat);
			if (fromFile.eof()){
				cout << "end of file!";
				break;
			}
		}
		getline_t(&toFile, &toMat);
		//process from marker data
		if(toFile.eof()) {
			cout << "end of file!";
			break;
		}
		helper.getCameraPose(&fromMat, &toMat, &toCampose);
		bool invalidMarkerPose = false;
		for (int i = 0; i < 8; i++) {
			float test = toCampose.at<float>(i, 0);
			if (test != test) {
				invalidMarkerPose = true;
			} else if (abs(test) > LIMIT1 ) {
				invalidMarkerPose = true;
			}
		}
		if (!invalidMarkerPose) {
			for (int j = 0; j < 8; j++){
				// acc[i].push_back(toCampose.at<float>(i,0));	
				if (j < 3) {
					// gsl_histogram_increment(h[j],toCampose.at<float>(j,0));
					acc[j].push_back(toCampose.at<float>(j,0));
				}
				if ((j >= 3)&&(j <= 5)) {
					int index = 2*(j-3) + 3;
					acc[index].push_back(sin(toCampose.at<float>(j,0)));
					acc[index + 1].push_back(cos(toCampose.at<float>(j,0)));
					// sum [index] += sin(acc[j].at(i));
					// sum [index+1] += cos(acc[j].at(i));

					// gsl_histogram_increment(h[index],sin(toCampose.at<float>(j,0)));
					// gsl_histogram_increment(h[index+1],cos(toCampose.at<float>(j,0)));
				}
				if (j > 5) {
					// sum [j+3] += acc[j].at(i);
					acc[j+3].push_back(toCampose.at<float>(j,0));
					// gsl_histogram_increment(h[j+3],sin(toCampose.at<float>(j,0)));
				}
			}
			fp 	<< toCampose.at<float>(0, 0) 
			<< "," << toCampose.at<float>(1, 0)
			<< "," << toCampose.at<float>(2, 0) << ","
			<< toCampose.at<float>(3, 0) << ","
			<< toCampose.at<float>(4, 0) << ","
			<< toCampose.at<float>(5, 0) << ","
			<< toCampose.at<float>(6, 0) << ","
			<< toCampose.at<float>(7, 0) <<  "\n";
			count++;
			// cout << "count: " << count << endl << "campose" << endl<<toCampose << endl<<endl;
		}
		if ( count == max_samples) break;
		
	}

	// rejectOutlier2(h[0]);

	float result[8];
	result[0] = BucketSort(acc[0],LIMIT1);
	result[1] = BucketSort(acc[1],LIMIT1);
	result[2] = BucketSort(acc[2],LIMIT1);
	result[3] = atan2(BucketSort(acc[3],LIMIT2), BucketSort(acc[4],LIMIT2));
	result[4] = atan2(BucketSort(acc[5],LIMIT2), BucketSort(acc[6],LIMIT2));
	result[5] = atan2(BucketSort(acc[7],LIMIT2), BucketSort(acc[8],LIMIT2));
	result[6] = BucketSort(acc[9],LIMIT3);
	result[7] = BucketSort(acc[10],LIMIT3);

	
	// if (rejectOutlier(acc, result) == 0){
	cout << "done!" << endl;
	fp.close();


	Mat res = (Mat_<float>(8,1) << result[0],result[1], result[2], result[3],
		result[4], result[5], result[6], result[7]);
	// // cout << "done!" << endl;

	saveCamPose(res, resultPath.c_str()); 
	cout << "[End Debug] cam_pose'= " << endl << res << endl << endl;
	// } else {
	// 	cout << "invalid data. Camera pose not computed"<<endl;
	// }
	
	return 0;
}


