#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>

int no_of_coeffs;
int batch_size = 512;



int main(int argc, char *argv[])
{	
	// assert(argc == 2);
	// char *fp = argv[1];
	// no_of_coeffs = get_coeffs_count(fp);
	// printf("%d\n",no_of_coeffs);
	// double *coeffs;
	// // Put coeffs in buffer
	// get_coeffs_from_file(fp, coeffs);

	// // Buffer containing one sample (left and right, both 16 bit).
	// int16_t samples[2*(batch_size+no_of_coeffs - 1)];
	// int16_t outputBuffer[2*batch_size];

	// unsigned sampleBufferSize=sizeof(samples);	// size in bytes of 
	// unsigned outputBufferSize=sizeof(outputBuffer);	// size in bytes of 

	// // Initially fill entire buffer.
	// int got=read(STDIN_FILENO, samples, sampleBufferSize);
	float count = 0;
	while(1){
		// Read one sample from input
		count--;
		float data[2] = {count, count-1};
		// if(got<0){
		// 	fprintf(stderr, "%s : Read from stdin failed, error=%s.", argv[0], strerror(errno));
		// 	exit(1);
		// }else if(got==0){
		// 	break;	 // end of file
		// }else if(got!=sampleBufferSize){
		// 	fprintf(stderr, "%s : Did not receive expected number of bytes.\n", argv[0]);
		// 	exit(1);
		// }
		// fir_filter(samples, outputBuffer, coeffs);
		// // Copy one sample to output
		int done=write(STDOUT_FILENO, data, 2*sizeof(float));
		 usleep(100000);
		// if(done<0){
		// 	fprintf(stderr, "%s : Write to stdout failed, error=%s.", argv[0], strerror(errno));
		// 	exit(1);
		// }else if(done!=outputBufferSize){
		// 	fprintf(stderr, "%s : Could not read requested number of bytes from stream.\n", argv[0]);
		// }

		// // Perform some recycling here.
		// // Copy contents backwards
		// for (int i = 0; i < batch_size; i++) {
		// 	samples[2*i] = samples[2*(no_of_coeffs+i-1)];
		// 	samples[2*i + 1] = samples[2*(no_of_coeffs+i-1) + 1];
		// }
		// got=read(STDIN_FILENO, samples+2*(no_of_coeffs-1), outputBufferSize);
		
	}
	return 0;
}
