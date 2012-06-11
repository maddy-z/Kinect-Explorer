#include <cstdio>
#include <cstdlib>

#include <opencv2\opencv.hpp>

#define		IMAGE_NAME				"lena.png"
#define		IMAGE_WIN_NAME		"Image Pyramid Demo"

// 
// Global Variables
// 

cv::Mat srcImg;
cv::Mat tmpImg;
cv::Mat dstImg;

// 
// Main Function
// 

int main ( int argc, char ** argv )
{
	srcImg = cv::imread ( IMAGE_NAME );
	
	if ( !srcImg.data ) 
	{
		printf("No data ! -- Exiting the program\n");

		system("pause");
		return -1;
	}

	tmpImg = srcImg;
	// dstImg = tmpImg;

	// 
	// Create Window
	// 
	cv::namedWindow(IMAGE_WIN_NAME);
	cv::imshow(IMAGE_WIN_NAME, tmpImg);

	while ( true ) 
	{
		int c;
		c = cv::waitKey(10);

		if ( (char)(c) == 'e' ) {
			break;
		}
		else if ( (char)(c) == 'u') {
			cv::pyrUp(tmpImg, tmpImg, cv::Size(tmpImg.cols * 2, tmpImg.rows * 2));
			printf ("Zoom In\n");
		}
		else if ( (char)(c) == 'd') {
			cv::pyrDown(tmpImg, tmpImg, cv::Size(tmpImg.cols / 2, tmpImg.rows / 2));
			printf ("Zoom Out\n");
		}

		cv::imshow(IMAGE_WIN_NAME, tmpImg);
	}

	return 0;
}