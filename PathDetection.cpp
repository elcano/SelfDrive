// LineDetectionMain.cpp : Defines the entry point for the console application.
//
/*
Author: Conor Van Achte
Created: 05/07/2018
Last Modified: 05/16/2018
Notes: Paved path detection for the Elcano project. This code uses a Canny edge detector with a Hough Lines Transform P to find edges
       and detect the edges of the paved path. This method will find a lot of noise currently, but should do reasonable well to 
	   give us a basic paved path detection algorithm. OpenCV is a main library used for the techniques found in the code and may need
	   to be downloaded onto your PC to be able to compile the program. A link to help setup opencv with visual studio can be found
	   here: https://youtu.be/l4372qtZ4dc.


*/


#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <math.h>
#include <queue> 

//extern "C" {
//#include "rs232.h"
//}


#define DEG_PER_PIXEL		0.036
#define BUF_SIZE 		128

using namespace cv;
using namespace std;

const Scalar light_green = Scalar(100, 255, 0);
const Scalar yellow = Scalar(0, 255, 255);
const Scalar red = Scalar(0, 0, 255);
int cport_nr = 24; /* /dev/ttyACM0 */


void detectLines(Mat& imgOriginal, int processCount, queue<Point> &leftWindow, queue<Point> &rightWindow);
double getLength(Point p1, Point p2);
int getShiftAmount(int x);
void addToLine(Point a, Point b, Point &top, Point &bot, queue<Point> &window);
void averagePoints(Point &a, Point &b, int numLines);
Mat rotate(Mat src, double angle);
vector<Point> getLatestLine(queue<Point> &q);
vector<Point> getMovingAvg(queue<Point> &q);


// These two methods are for use when communicating with the arduino mega the raspberry pi will be connected to.
//void sendToArduino(float dist, float deg);
//void receiveFromArduino();




int main(int argc, char* argv[]) {
	int bdrate = 57600; /* 9600 baud */

	char mode[] = { '8','N','1',0 }; // 8 data bits, no parity, 1 stop bit

									 //if (RS232_OpenComport(cport_nr, bdrate, mode)) {
									 //	cout << "Can not open comport\n";
									 //    return 0;
									 //}
									 //Sleep(2000000);
									 //usleep(2000000);  /* waits 2000ms for stable condition */


	
	
    //VideoCapture capture(0);
	VideoCapture capture("outside13.mp4");
	Mat frame;
	// Some of the test videos are upside down so we use this when testing on those videos
	Mat flipped;

	bool found = false;

	int processCount = 0;
	// Queues are used for a rolling window of values. Information on the previous lines are stored here
	// 5/16/18 The code currently places the top point in the first value of the queue, and the bottom point as the second value
	queue<Point> leftWindow;
	queue<Point> rightWindow;
	while (true) {

		
		capture >> frame;

		// Rotate image 180 degrees so it is rightside up
		flipped = rotate(frame, 180);
		// Draws a black rectangle on top half of image. Lines found on the ground are what we are interested in
		//rectangle(flipped, Point(0, 0),
		//	Point(640, 240), Scalar(0, 0, 0), CV_FILLED, 8);

		imshow("flipped", flipped);
		
		//Mat dst;
		//frame = flipped;
		/// Convert to grayscale
		//cvtColor(frame, frame, CV_BGR2GRAY);


		
		
		detectLines(flipped, processCount, leftWindow, rightWindow);
		// The following two lines allow you to pause the video indefintely by pressing "p", and allows you to resume the video by pressing "p".
		if (cv::waitKey(1) == 'p')
			while (cv::waitKey(1) != 'p');
		processCount++;
		if (waitKey(20) == 27)
			break;
	}

	capture.release();
	return 0;
}



void detectLines(Mat& input, int processCount, queue<Point> &leftWindow, queue<Point> &rightWindow) {
	//cout << processCount << endl;
	if (input.empty()) {
		cout << "Error loading the image" << endl;
		exit(1);
	}

	if (processCount > 40) {
		Mat img_gray;
		cvtColor(input, img_gray, CV_BGR2GRAY);
		int thresh = 50;
		Mat input_blur;
		GaussianBlur(input, img_gray, Size(3, 3), 0, 0);

		Mat canny_output;



		/// Detect edges using canny
		Canny(img_gray, canny_output, thresh, thresh * 3, 3);
		rectangle(canny_output, Point(0, 0),
			Point(640, 240), Scalar(0, 0, 0), CV_FILLED, 8);

		//imshow("input_blur", input_blur);
		imshow("canny output", canny_output);
		// Create a vector which contains 4 integers in each element (coordinates of the line)
		// This vector will be used the output of the Hough Lines transform
		vector<Vec4i> lines;

		// Set limits on hough trasnform line detection
		//Good value here
		//double minLineLength = 80;
		double minLineLength = 120;
		double maxLineGap = 5;

		HoughLinesP(canny_output, lines, 1, CV_PI / 180, 80, minLineLength, maxLineGap);

		int numLines = lines.size();
		cout << "numLines = " << numLines << endl;
		double longestLine = 0;

		Point highestPoint(0, 0);
		Point lowestPoint(0, 0);
		Point topL, botL, topR, botR;
		int numLeft = 0;
		int numRight = 0;
		// Draws the lines found from the houghline transform
		for (size_t i = 0; i < numLines; i++) {
			float x1 = lines[i][0];
			float y1 = lines[i][1];
			float x2 = lines[i][2];
			float y2 = lines[i][3];

			// Get the slopes of the lines detected so we can filter the lines detected by the Hough Transform
			float slope = 0;
			if ((x2 - x1) != 0) {
				slope = (y2 - y1) / (x2 - x1);
			}

			int length = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));

			if (slope != 0 && slope < 2) {
				if ((x1 < 320) && (x2 < 320)) {
					// Line on left 
					cout << "Adding line = " << "(" << x1 << ", " << y1 << ")" << " (" << x2 << ", " << y2 << ")" << endl;
					addToLine(Point(x1, y1), Point(x2, y2), topL, botL, leftWindow);
					numLeft++;
				}
				else if ((x1 > 320) && (x2 > 320)) {
					// Line on right
					addToLine(Point(x1, y1), Point(x2, y2), topR, botR, rightWindow);
					numRight++;
				}
				//line(input, Point(x1, y1),
				//	Point(x2, y2), Scalar(156, 25, 0), 3, 8);
			}
		}
		

		int size = leftWindow.size();
		cout << "Left window size = " << size << endl;
		cout << "Printing contents of leftWindow..." << endl;
		for (int i = 0; i < size; i++) {
			int xVal = leftWindow.front().x;
			int yVal = leftWindow.front().y;
            leftWindow.pop();
            cout << "(" << xVal << ", " << yVal << ")" << endl;
            leftWindow.push(Point(xVal, yVal));
		}
		cout << "Done...." << endl;
		//cout << "rightWindow = " << rightWindow << endl;
		// 5/16/18 Somewhat bad coding practice here, refactoring should occur for this sometime soon but this will work for now
		vector<Point> getPrevLine = getLatestLine(leftWindow);
		cout << "getPrevLine size = " << getPrevLine.size() << endl;

		if (leftWindow.size() > 2) {
			cout << "Inside" << endl;
			vector<Point> movingAvg = getMovingAvg(leftWindow);
			line(input, movingAvg[0],
				movingAvg[1], Scalar(20, 100, 100), 3, 8);
		}
		else {
			line(input, topL,
				botL, Scalar(156, 25, 0), 3, 8);
		}

		if (numRight == 0) {
			vector<Point> getPrevLine = getLatestLine(rightWindow);
			if (getPrevLine.size() == 2) {
				line(input, getPrevLine[0],
					getPrevLine[1], Scalar(0, 25, 156), 3, 8);
			}
		}

		line(input, topR,
			botR, Scalar(156, 25, 0), 3, 8);

		imshow("input", input);
	}
}


// Adds the first two paramaters to the top or bot points that are passed in. This is to get a running average of
// points for the detected line. The method also takes in a queue of points which
// are going to be used for 
void addToLine(Point a, Point b, Point &top, Point &bot, queue<Point> &window) {
	//cout << "a = " << a << " b = " << b << endl;
	if (a.y > b.y) {
		bot.x += a.x;
		bot.y += a.y;
		top.x += b.x;
		top.y += b.y;
	}
	else {
		bot.x += b.x;
		bot.y += b.y;
		top.x += a.x;
		top.y += a.y;
	}
	window.push(top);
	window.push(bot);
}

void averagePoints(Point &a, Point &b, int numLines) {

	if (numLines != 0) {
		a.x /= numLines;
		a.y /= numLines;
		b.x /= numLines;
		b.y /= numLines;
	}

}

int getShiftAmount(int x) {
	// We want to shift the x into the middle to figure out the angle that would be made between these two lines
	int shiftTarget = 320;
	return shiftTarget - x;
}

double getLength(Point p1, Point p2) {
	int xVal = pow(p2.x - p1.x, 2);
	int yVal = pow(p2.y - p1.y, 2);
	int distance = sqrt(xVal + yVal);
	return distance;
}

vector<Point> getMovingAvg(queue<Point> &q) {
	vector<Point> ret;
	int size = q.size() / 2;
	Point top(0, 0);
	Point bot(0, 0);

	for (int i = 0; i < size; i++) {
		top.x += q.front().x;
		top.y += q.front().y;
		q.pop();
		bot.x += q.front().x;
		bot.y += q.front().y;
		q.pop();
	}
	top.x /= size;
	top.y /= size;
	bot.x /= size;
	bot.y /= size;
	cout << "top = " << top << " bot = " << bot << endl;
	if (q.size() <= 10) {
	    ret.push_back(top);
	    ret.push_back(bot);
    }
	

	return ret;
}

vector<Point> getLatestLine(queue<Point> &q) {
	vector<Point> ret;
	int size = q.size();
	// Check if queue is empty
	if (size > 0 && size >= 2) {
		Point top(q.front().x, q.front().y);
		q.pop();
		Point bot(q.front().x, q.front().y);
		q.pop();
		if (size <= 20) {
			q.push(top);
			q.push(bot);
		}
		ret.push_back(top);
		ret.push_back(bot);
	}
	return ret;
}

Mat rotate(Mat src, double angle)
{
	if (src.empty()) {
		cout << "Error loading the image" << endl;
		exit(1);
	}
	Mat dst;
	Point2f pt(src.cols / 2., src.rows / 2.);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(src, dst, r, Size(src.cols, src.rows));
	return dst;
}


//void sendToArduino(float dist, float deg) {
//	int i_deg = (int)deg + 10;
//	if (i_deg < 0) i_deg = 0;
//	string send_str = to_string((int)dist) + "," + to_string(i_deg) + "\n";
//	RS232_cputs(cport_nr, send_str.c_str());
//	cout << "Sent to Arduino: " << send_str;
//	//usleep(8000);
//	Sleep(8000);
//}
//
//void receiveFromArduino() {
//	unsigned char str_recv[BUF_SIZE]; // recv data buffer
//	int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
//	if (n > 0) {
//		str_recv[n] = 0;   // put null at end
//		cout << "Received " << n << " bytes: " << (char *)str_recv << "\n";
//	}
//}
