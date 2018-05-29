#ifndef CAMSHIFT_H
#define CAMSHIFT_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Mat image;
bool backprojMode = false; 
bool selectObject = false;
int trackObject = 0; 
bool showHist = true;
Point origin;
Rect selection;

int vmin = 10, vmax = 256, smin = 30;
int diff;
int data=4;

Rect trackWindow;
RotatedRect trackBox;

int hsize = 60;
float hranges[] = {0,180};
const float* phranges = hranges;
Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj,bkpj;

#endif
