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


		
};