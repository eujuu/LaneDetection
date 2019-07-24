#include <opencv2/core/core.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <Windows.h>
#include <winuser.h>
#include <stdlib.h> 
#include <string.h> 
#include <tchar.h>
#include <vector>
using namespace cv;
using namespace std;

#define PI 3.1415926

int line_num = 0;
int is_LaneDetected = 0;
int is_LaneChanged = 0;
int first_flag = 0;

class LineDetector {
private:
	Mat img;
	vector<Vec4i> lines;

	double deltaRho;
	double deltaTheta;
	int minVote;
	double minLength;
	double maxGap;
public:
	int count;

	LineDetector() : deltaRho(1), deltaTheta(PI / 180), minVote(10), minLength(0.), maxGap(0.) {}

	void settings(double dRho, double dTheta, int minv, double length, double gap) {
		deltaRho = dRho;
		deltaTheta = dTheta;
		minVote = minv;
		minLength = length;
		maxGap = gap;
	}

	vector<Vec4i> findLines(Mat& binary) {
		lines.clear();
		HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		return lines;
	}
	vector<Point> Detect_points(Mat &image, int x, int y, Scalar color = Scalar(255, 0, 0)) {
		vector<Vec4i>::const_iterator it2 = lines.begin();
		vector<Point> line_point;

		while (it2 != lines.end()) {
			Point pt1((*it2)[0] + x, y + (*it2)[1]);
			Point pt2((*it2)[2] + x, y + (*it2)[3]);

			if (((*it2)[0] - (*it2)[2]) != 0) {
				if (double(((*it2)[1] - (*it2)[3]) / ((*it2)[0] - (*it2)[2])) > 0.1 | double(((*it2)[1] - (*it2)[3]) / ((*it2)[0] - (*it2)[2])) < -0.1) {
					line_point.push_back(pt1);
					line_point.push_back(pt2);
				}
			}
			++it2;
		}
		return line_point;
	}
};
String clip;
vector<Point> linesetting(Mat &frame, Mat &Contour, int a, int b, int image_x) {
	Mat out_img = frame.clone();
	vector<Point> output(2);
	vector<Point> line_point;
	Vec4d out_point;
	LineDetector ld;
	ld.settings(1, PI / 180, 10, 20, 30);

	vector<Vec4i> li = ld.findLines(Contour);
	line_point = ld.Detect_points(frame, a, b);
	if (line_point.size() > 0) {
		fitLine(line_point, out_point, CV_DIST_L2, 0, 0.01, 0.01);
		int x1 = ((image_x)-out_point[3]) / out_point[1] * out_point[0] + out_point[2];
		int y1 = image_x - 1;
		int x2 = ((image_x / 2) - out_point[3]) / out_point[1] * out_point[0] + out_point[2];
		int y2 = (image_x / 2);
		output[0] = Point(x1, y1);
		output[1] = Point(x2, y2);
		if (abs(x2 - x1) > 300) line_num = 2;
	}
	return output;
}

Mat lane_changed(Mat origin, double ratio) {
	string change = "";
	if (ratio <= 93) {
		change = "Left";
		is_LaneDetected = 0;
		line_num = 1;
	}
	else if (ratio > 108) {
		change = "right";
		is_LaneDetected = 0;
		line_num = 1;
	}
	else {
		change = "Straight";
	}
	putText(origin, change, Point(50, 90), FONT_HERSHEY_PLAIN, 3, Scalar(127, 0, 255), 1, CV_AA);
	return origin;
}


Mat PreProcessing(Mat& ROI) {
	Mat whitelane;
	Mat output;

	Scalar MinScalar = Scalar(110, 70, 50);
	if (clip == "clip2.mp4" || clip == "clip3.mp4")
		MinScalar = Scalar(80, 50, 30);
	Scalar MaxScalar = Scalar(255, 255, 255);

	inRange(ROI, MinScalar, MaxScalar, whitelane);

	threshold(whitelane, output, 150, 255, THRESH_BINARY);

	medianBlur(output, output, 7);

	return output;
}

int main()
{
	VideoCapture capture;
	vector<Point>left(2), right(2);
	Mat left_m, right_m;
	Mat frame, ROI, ROI1, ROI2;
	vector <Point> left_before(2), right_before(2);
	vector <Point> first_left(2), first_right(2);
	clip = "clip1.mp4";
	capture.open(clip);
	double ratio = 100;
	if (!capture.isOpened())
		return -1;
	namedWindow("LaneDetect", 1);
	int npts;
	int frame_cnt = 0; // 프레임개수 카운팅




	char ratioText[50];

	for (;;) {
		capture >> frame;
		resize(frame, frame, Size(640, 480));
		ROI = frame(Rect(110, 350, 320, 120)); //clip1,2,3
		int ROI1_x = 110, ROI1_y = 350;
		int ROI2_x = 280, ROI2_y = 350;
		ROI1 = frame(Rect(110, 350, 160, 120));
		ROI2 = frame(Rect(280, 350, 160, 120));



		if (clip == "clip4.mp4") {
			ROI = frame(Rect(100, 250, 400, 100));
			ROI1_x = 100, ROI1_y = 250;
			ROI2_x = 300, ROI2_y = 250;
			ROI1 = frame(Rect(100, 250, 200, 100));
			ROI2 = frame(Rect(300, 250, 200, 100));
		}

		left_m = PreProcessing(ROI1);
		right_m = PreProcessing(ROI2);
		frame = lane_changed(frame, ratio);

		//imshow("Pre1", ROI1);
	//	imshow("Pre2", ROI2);

		if (is_LaneDetected == 0 || line_num < 2) {
			left_before = left;
			right_before = right;

			left = linesetting(frame, left_m, ROI1_x, ROI1_y, 640);
			right = linesetting(frame, right_m, ROI2_x, ROI2_y, 640);

			is_LaneDetected = 1;
			frame_cnt = 0;
		}

		if (first_flag == 0) {
			first_left = left;
			first_right = right;
			first_flag = 1;
		}

		if (is_LaneDetected == 1 && frame_cnt == 20) {
			is_LaneDetected = 0;
			frame_cnt = 0;
		}

		if (frame_cnt == 0 && line_num == 2) {
			if (left[0] == Point(0, 0) || left[1] == Point(0, 0)) {
				left = left_before;
			}
			if (right[0] == Point(0, 0) || right[1] == Point(0, 0)) {
				right = right_before;
			}
		}
		line(frame, left[0], left[1], Scalar(0, 0, 255), 5, CV_AA);
		line(frame, right[0], right[1], Scalar(0, 0, 255), 5, CV_AA);
		vector< Point> contour;

		contour.push_back(left[0]);
		contour.push_back(left[1]);
		contour.push_back(right[1]);
		contour.push_back(right[0]);

		Mat frame_last;
		const Point *pts = (const cv::Point*) Mat(contour).data;

		npts = Mat(contour).rows;
		frame.copyTo(frame_last);

		line(frame, (first_right[1] + first_left[1]) / 2 + Point(0, 80), (first_right[0] + first_left[0]) / 2, Scalar(0, 0, 0), 5, CV_AA);

		if (line_num == 2 && left[1] != Point(0, 0) && left[0] != Point(0, 0) && right[0] != Point(0, 0) && right[1] != Point(0, 0)) {
			fillPoly(frame_last, &pts, &npts, 1, Scalar(0, 255, 0), 8);
			addWeighted(frame_last, 0.3, frame, 0.7, 0, frame);
			line(frame, (right[1] + left[1]) / 2 + Point(0, 120), (right[0] + left[0]) / 2, Scalar(255, 255, 255), 5, CV_AA);

			Point origin = (first_right[1] + first_left[1]) / 2;
			Point present = (right[1] + left[1]) / 2;

			ratio = 100 * origin.x / present.x;
			sprintf_s(ratioText, "%.1f%%", ratio);
			putText(frame, ratioText, Point(250, 290), FONT_HERSHEY_PLAIN, 3, Scalar(0, 255, 255), 1, CV_AA);
		}

		imshow("LaneDetect", frame);
		frame_cnt++;
		if (waitKey(30) >= 0) break;
	}

	return 0;
}