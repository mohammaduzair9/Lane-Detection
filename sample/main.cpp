#include<opencv2\opencv.hpp>
#include<opencv2\imgcodecs.hpp>
#include "LaneDetection.cpp"
#include<iostream>

using namespace std;
using namespace cv;

int main() {

	VideoCapture cap("project_video.mp4");
	if (!cap.isOpened())
		return -1;

	Mat original_frame;
	Mat video_frame;
	vector<Point> laneVertices;

	LaneLineDetector lanelinedetector;
	while (cap.read(original_frame)) {
		
		video_frame = lanelinedetector.change_in_grayscale(original_frame);
		video_frame = lanelinedetector.detect_yellow_white(video_frame);
		video_frame = lanelinedetector.clean_image(video_frame);
		video_frame = lanelinedetector.detect_edges(video_frame);
		video_frame = lanelinedetector.crop_image_to_roi(video_frame);
		laneVertices = lanelinedetector.generate_laneLines(video_frame);
		if (laneVertices[0] == Point(0, 0)  &&
			laneVertices[1] == Point(0, 0) &&
			laneVertices[2] == Point(0, 0) &&
			laneVertices[3] == Point(0, 0)) {
			
			continue;
			
		}
		else {
			lanelinedetector.drawLanes(laneVertices, original_frame);
			waitKey(5);
		}
	}

	return 0;
}
