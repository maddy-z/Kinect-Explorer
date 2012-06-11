#include <cstdio>
#include <cstdlib>

#include <opencv2\opencv.hpp>

#define		SRC_IMAGE_WIN_NAME				"Original Image"
#define		CANNYED_IMAGE_WIN_NAME			"Cannyed Image"

#define		TRACKBAR_ONE					"Threshold 1"
#define		TRACKBAR_TWO					"Threshold 2"

// 
// Global cv::Mat
// 

cv::Mat srcImg;
cv::Mat cannyedImg;

int threshold_1 = 0;
int threshold_2 = 0;

void thresholdChanged_1 ( int pos, void * )
{
	// 
	// Canny Operation
	// 

	cv::Canny(srcImg, cannyedImg, pos, threshold_2);
	cv::imshow(CANNYED_IMAGE_WIN_NAME, cannyedImg);

	return;
}

void thresholdChanged_2 ( int pos, void * )
{
	// 
	// Canny Operation
	// 

	cv::Canny(srcImg, cannyedImg, threshold_1, pos);
	cv::imshow(CANNYED_IMAGE_WIN_NAME, cannyedImg);

	return;
}

int main(int argc, char ** argv)
{
	const char * filename = "lena.png";
	
	srcImg = cv::imread(filename, 0);
	cannyedImg = srcImg.clone();

	cv::namedWindow(SRC_IMAGE_WIN_NAME, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(CANNYED_IMAGE_WIN_NAME, CV_WINDOW_AUTOSIZE);

	cv::createTrackbar(TRACKBAR_ONE, CANNYED_IMAGE_WIN_NAME, &threshold_1, 600, thresholdChanged_1);
	cv::createTrackbar(TRACKBAR_TWO, CANNYED_IMAGE_WIN_NAME, &threshold_2, 600, thresholdChanged_2);

	// 
	// Show Original Image
	// 
	
	cv::Canny(srcImg, cannyedImg, threshold_1, threshold_2);

	cv::imshow(SRC_IMAGE_WIN_NAME, srcImg);
	cv::imshow(CANNYED_IMAGE_WIN_NAME, cannyedImg);

	cvWaitKey(0);			//µÈ´ý°´¼ü

	return 0;
}