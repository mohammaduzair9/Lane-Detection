#include<opencv2\opencv.hpp>
#include<opencv2\imgcodecs.hpp>
#include "LaneDetection.cpp"
#include<iostream>

using namespace std;
using namespace cv;

int main() {

	//loading the video for lane detection
	VideoCapture cap("project_video.mp4");
	if (!cap.isOpened())
		return -1;

	Mat original_frame;
	Mat video_frame;
	vector<Point> laneVertices;

	// declaring the lane detector object
	LaneLineDetector lanelinedetector;
	
	//loop for reading the frames of the video
	while (cap.read(original_frame)) {
		
		//changing the frame in gray scale
		video_frame = lanelinedetector.change_in_grayscale(original_frame);
		
		//detecting the yellow and white colors in the frame
		video_frame = lanelinedetector.detect_yellow_white(video_frame);
		
		//removing the noise from the frame
		video_frame = lanelinedetector.clean_image(video_frame);
		
		//detecting the edges in the frame
		video_frame = lanelinedetector.detect_edges(video_frame);
		
		//cropping to get the region of interest
		video_frame = lanelinedetector.crop_image_to_roi(video_frame);
		
		// getting the verices of the lanes
		laneVertices = lanelinedetector.generate_laneLines(video_frame);

		//if lane veritces are not detected then go to next frame
		if (laneVertices[0] == Point(0, 0)  &&
			laneVertices[1] == Point(0, 0) &&
			laneVertices[2] == Point(0, 0) &&
			laneVertices[3] == Point(0, 0)) {
			
			continue;
			
		}
		//if vertices are detected then plot them on the original image
		else {
			lanelinedetector.drawLanes(laneVertices, original_frame);
			
			//waiting for the image to get displayed
			waitKey(5);
		}
	}

	return 0;
}
