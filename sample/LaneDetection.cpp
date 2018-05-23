#pragma once

#include <vector>
#include <string>
#include<iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class LaneLineDetector {

	private:
		
	public:

		Mat change_in_grayscale(Mat image) {

			Mat gray_image;
			cvtColor(image, gray_image, COLOR_BGR2GRAY);

			return gray_image;
		}
		
		Mat clean_image(Mat image) {
		
			Mat cleaned_image;
			Size blur_kernel_size(5,5);
			GaussianBlur(image, cleaned_image, blur_kernel_size, 0, 0);

			return cleaned_image;
		}

		Mat detect_yellow_white(Mat image) {
		
			Scalar yellow_low = Scalar(0, 60, 60);
			Scalar yellow_high = Scalar(30, 255, 255);

			Mat mask1, mask2, mask3;
			Mat yelwhite;
			
			inRange(image, yellow_low, yellow_high, mask1);
			
			inRange(image, 200, 255, mask2);

			bitwise_or(mask1, mask2, mask3);

			bitwise_and(image, mask3, yelwhite);

			return yelwhite;
		}
		

		
};