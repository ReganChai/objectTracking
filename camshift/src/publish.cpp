#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "../include/camshift/init.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>

static const std::string OPENCV_WINDOW = "Camshift Demo";
cv_bridge::CvImagePtr cv_ptr;


static void onMouse( int event, int x, int y, int, void* )
{
  if( selectObject )
    {
	  selection.x = MIN(x, origin.x);
	  selection.y = MIN(y, origin.y);
	  selection.width = std::abs(x - origin.x);
	  selection.height = std::abs(y - origin.y);
	  selection &= Rect(0, 0, image.cols, image.rows);
    } 
  switch( event )
    {
	  case CV_EVENT_LBUTTONDOWN:
	       origin = Point(x,y);
	       selection = Rect(x,y,0,0);
	       selectObject = true;
	  break;
	  case CV_EVENT_LBUTTONUP:
	       selectObject = false;
		   if( selection.width > 0 && selection.height > 0 )
		   trackObject = -1;
	  break;
    }
}


int camshift()
{ 
   cvtColor(image, hsv, CV_BGR2HSV);    
   if (trackObject)
   {
	   int _vmin = vmin, _vmax = vmax;
	   inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	   int ch[] = { 0, 0 };
	   hue.create(hsv.size(), hsv.depth());
	   mixChannels(&hsv, 1, &hue, 1, ch, 1);
	   if (trackObject < 0)
	   {
		   Mat roi(hue, selection), maskroi(mask, selection);
		   calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
		   normalize(hist, hist, 0, 255, CV_MINMAX);
		   trackWindow = selection;
		   trackObject = 1;
		   histimg = Scalar::all(0);
		   int binW = histimg.cols / hsize;
		   Mat buf(1, hsize, CV_8UC3);
		   for (int i = 0; i < hsize; i++)
			   buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
		   cvtColor(buf, buf, CV_HSV2BGR);
	   }
	   calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
	   backproj &= mask;
	   RotatedRect trackBox = CamShift(backproj, trackWindow, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
	   if (trackWindow.area() <= 1)
	   {
		   int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
		   trackWindow = Rect(trackWindow.x - r, trackWindow.y - r, trackWindow.x + r, trackWindow.y + r) &Rect(0, 0, cols, rows);
	   }

	   if (backprojMode)
		   cvtColor(backproj, image, CV_GRAY2BGR);
	   ellipse(image, trackBox, Scalar(255, 255, 255), 3, CV_AA);
	   int width = image.cols / 2;
	   diff = width - trackBox.center.x;
	   if ((-300 < diff) && (diff < -240))
		   data = 0;
	   else if ((-240 < diff) && (diff < -180))
		   data = 1;
	   else if ((-180 < diff) && (diff < -120))
		   data = 2;
	   else if ((-120 < diff) && (diff < -60))
		   data = 3;
	   else if ((-60 < diff) && (diff < 0))
		   data = 4;
	   else if ((0 < diff) && (diff < 60))
		   data = 5;
	   else if ((60 < diff) && (diff < 120))
		   data = 6;
	   else if ((120 < diff) && (diff < 180))
		   data = 7;
	   else if ((180 < diff) && (diff < 240))
		   data = 8;
	   //else if((240<diff)&&(diff<300))
		   //data=9;

   }
   if(selectObject)
     rectangle(image,selection,Scalar(255,255,255),3,8,0);
  
     imshow( OPENCV_WINDOW,image );   
     waitKey(10);
     
     return 0;
}


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
public:
	ImageConverter()
		:it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);

		image_pub_ = it_.advertise("/image_converter/output_video", 1);
		namedWindow(OPENCV_WINDOW);

		setMouseCallback(OPENCV_WINDOW, onMouse, 0);

	}
	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		// cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			//cv::setMouseCallback(OPENCV_WINDOW, onMouse,0);

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Update GUI Window
		// cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		image = cv_ptr->image;
		camshift();

		// cv::waitKey(3);     
		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle n;
	ImageConverter ic;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << data;

		msg.data = ss.str();
		chatter_pub.publish(msg);
		ROS_INFO("%s", msg.data.c_str());

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}




