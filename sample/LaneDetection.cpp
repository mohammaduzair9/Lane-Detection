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
		

		
};