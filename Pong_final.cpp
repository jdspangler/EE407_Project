//-----------------------------------------------------------------------------------------------------------------
// File:   Lab6.cpp
// Author: Jordan Spangler, based on original code by Allan A. Douglas
//-----------------------------------------------------------------------------------------------------------------

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string> 
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <opencv2/cudacodec.hpp>
#include <leastSquare.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

// Select Video Source
// The MP4 demo uses a ROI for better tracking of the moving object
//#define TEST_LIVE_VIDEO

using namespace cv;
using namespace std;
//initial min and max HSV filter values.
//HSV filter values for bottom paddle
int H_MIN_bot = 0;
int H_MAX_bot = 42;
int S_MIN_bot = 21;
int S_MAX_bot = 68;
int V_MIN_bot = 250;
int V_MAX_bot = 256;
//HSV filter values for top paddle
int H_MIN_top = 70;
int H_MAX_top = 111;
int S_MIN_top = 208;
int S_MAX_top = 256;
int V_MIN_top = 140;
int V_MAX_top = 256;
//HSV filter values for playing field
int H_MIN_fld = 0;
int H_MAX_fld = 256;
int S_MIN_fld = 0;
int S_MAX_fld = 256;
int V_MIN_fld = 94;
int V_MAX_fld = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
int bot_roi[4] = {520, 625, 308, 30};
int top_roi[4] = {520, 60, 308, 30};
int theObject[2] = {0,0};
int x = 0;
int y = 0;
cv::Rect roi(520, 90, 308, 535);
cv::Point2i point_input;
leastSquare lsf(roi);
//bounding rectangle of the object, we will use the center of this as its position.
cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}


//void createTrackbars(){
	//create window for trackbars

    //namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	//char TrackbarName[50];
	//sprintf( TrackbarName, "H_MIN", H_MIN);
	//sprintf( TrackbarName, "H_MAX", H_MAX);
	//sprintf( TrackbarName, "S_MIN", S_MIN);
	//sprintf( TrackbarName, "S_MAX", S_MAX);
	//sprintf( TrackbarName, "V_MIN", V_MIN);
	//sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    //createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    //createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    //createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    //createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    //createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    //createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );

//}

//-----------------------------------------------------------------------------------------------------------------
// Paddle Control Serial Interface function
//-----------------------------------------------------------------------------------------------------------------
void paddleCommand(char command)
{
  char write_buffer[1];
  char read_buffer[1];
  int  bytes_written;  
  int  bytes_read; 
  struct termios options;           // Terminal options
  int fd;                           // File descriptor for the port


  fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);   // Open tty device for RD and WR

  if(fd == 1) {
     printf("\n  Error! in Opening ttyUSB0\n");
  }
  else
     printf("\n  ttyUSB0 Opened Successfully\n");

    tcgetattr(fd, &options);               // Get the current options for the port
    cfsetispeed(&options, B115200);        // Set the baud rates to 115200          
    cfsetospeed(&options, B115200);                   
    options.c_cflag |= (CLOCAL | CREAD);   // Enable the receiver and set local mode           
    options.c_cflag &= ~PARENB;            // No parity                 
    options.c_cflag &= ~CSTOPB;            // 1 stop bit                  
    options.c_cflag &= ~CSIZE;             // Mask data size         
    options.c_cflag |= CS8;                // 8 bits
    options.c_cflag &= ~CRTSCTS;           // Disable hardware flow control    

// Enable data to be processed as raw input
    options.c_lflag &= ~(ICANON | ECHO | ISIG);
     
    tcsetattr(fd, TCSANOW, &options);      // Apply options immediately
    fcntl(fd, F_SETFL, FNDELAY);    

    write_buffer[0] = command;
    bytes_written  =  0 ;   
                                               
    bytes_written = write(fd, write_buffer, 1);

    if (bytes_written == -1)
	printf("W Error=%d\n", errno);
    else
        printf("Wrote=%c\n", write_buffer[0]);
 
    // Check response
    while(read(fd, &read_buffer, 1) != 1) {
    }

    printf("Read=%c\n", read_buffer[0]);

  close(fd);
}
//-----------------------------------------------------------------------------------------------------------------
// int to string helper function
//-----------------------------------------------------------------------------------------------------------------
string intToString(int number){
 
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}
//-----------------------------------------------------------------------------------------------------------------
// Corner Detector Function
//-----------------------------------------------------------------------------------------------------------------

void findCorners(cv::Mat src_gray, cv::Mat &cameraFeed) {
    cv::Mat dst, dst_norm;
    dst = cv::Mat::zeros( src_gray.size(), CV_32FC1 );
    int blockSize = 2;
    int apertureSize = 9;
    double k = 0.04;
    int thresh = 205;
    // Detecting corners
    cv::cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
    // Normalizing
    cv::normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    // Draw circles around courners
    for(int j = 0; j < dst_norm.rows; j++) {
	for(int i = 0; i < dst_norm.cols; i++) {
	    if( (int) dst_norm.at<float>(j,i) > thresh ) {
		cv::circle(cameraFeed, Point(i,j), 5, Scalar(0,255,0), 2, 8, 0);
	    }
	}
    }
}

//-----------------------------------------------------------------------------------------------------------------
// Search for Paddle
//-----------------------------------------------------------------------------------------------------------------
void searchForPaddle(cv::Mat thresholdImage, cv::Mat &cameraFeed, int roi_box[4]){
    //notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
    //to take the values passed into the function and manipulate them, rather than just working with a copy.
    //eg. we draw to the cameraFeed to be displayed in the main() function.

    bool objectDetected = false;
    int xpos, ypos, x1, y1;
    int thePaddle[2] = {0,0};
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Mat temp;

    thresholdImage.copyTo(temp);
#ifdef TEST_LIVE_VIDEO

    //find contours of filtered image using openCV findContours function
    cv::findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

#else
    
    cv::Rect roi(roi_box[0], roi_box[1], roi_box[2], roi_box[3]);
    //add region of interest box lines
    line(cameraFeed,Point(roi_box[0],roi_box[1]),Point(roi_box[0]+roi_box[2],roi_box[1]),Scalar(255,000,000),2);
    line(cameraFeed,Point(roi_box[0],roi_box[1]),Point(roi_box[0],roi_box[1]+roi_box[3]),Scalar(255,000,000),2);
    line(cameraFeed,Point(roi_box[0],roi_box[1]+roi_box[3]),Point(roi_box[0]+roi_box[2],roi_box[1]+roi_box[3]),Scalar(255,000,000),2);
    line(cameraFeed,Point(roi_box[0]+roi_box[2],roi_box[1]),Point(roi_box[0]+roi_box[2],roi_box[1]+roi_box[3]),Scalar(255,000,000),2);

    cv::Mat roi_temp = temp(roi); 

    //find contours of filtered image using openCV findContours function
    cv::findContours(roi_temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

#endif

    //if contours vector is not empty, we have found some objects
    if(contours.size()>0)
	objectDetected = true;
    else 
	objectDetected = false;
 
    if(objectDetected){

        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));

        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));

        xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
 
        //update the objects positions by changing the 'theObject' array values
        thePaddle[0] = xpos , thePaddle[1] = ypos;
    }

    //make some temp x and y variables so we dont have to type out so much

#ifdef TEST_LIVE_VIDEO
    x1 = thePaddle[0];
    y1 = thePaddle[1];

#else
    x1 = thePaddle[0]+roi_box[0];
    y1 = thePaddle[1]+roi_box[1];

#endif
//draw a box around the paddle
    //circle(cameraFeed,Point(x,y),10,cv::Scalar(0,255,0),2);
    line(cameraFeed,Point(x1-18,y1-10),Point(x1+18,y1-10),Scalar(0,255,0),2);
    line(cameraFeed,Point(x1+18,y1-10),Point(x1+18,y1+10),Scalar(0,255,0),2);
    line(cameraFeed,Point(x1+18,y1+10),Point(x1-18,y1+10),Scalar(0,255,0),2);
    line(cameraFeed,Point(x1-18,y1+10),Point(x1-18,y1-10),Scalar(0,255,0),2);
}
//-----------------------------------------------------------------------------------------------------------------
// Search for Moving Object
//-----------------------------------------------------------------------------------------------------------------
void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed){
    //notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
    //to take the values passed into the function and manipulate them, rather than just working with a copy.
    //eg. we draw to the cameraFeed to be displayed in the main() function.

    bool objectDetected = false;
    int xpos, ypos;

    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Mat temp;

    thresholdImage.copyTo(temp);

#ifdef TEST_LIVE_VIDEO

    //find contours of filtered image using openCV findContours function
    cv::findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

#else

    //cv::Rect roi(520, 90, 308, 535);
    //add region of interest box lines
    line(cameraFeed,Point(520,90),Point(828,90),Scalar(255,000,000),2);
    line(cameraFeed,Point(520,90),Point(520,625),Scalar(255,000,000),2);
    line(cameraFeed,Point(520,625),Point(828,625),Scalar(255,000,000),2);
    line(cameraFeed,Point(828,90),Point(828,625),Scalar(255,000,000),2);

    cv::Mat roi_temp = temp(roi); 

    //find contours of filtered image using openCV findContours function
    cv::findContours(roi_temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

#endif

    //if contours vector is not empty, we have found some objects
    if(contours.size()>0)
	objectDetected = true;
    else 
	objectDetected = false;
 
    if(objectDetected){

        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));

        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));

        xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
 
        //update the objects positions by changing the 'theObject' array values
        theObject[0] = xpos , theObject[1] = ypos;
    }

    //make some temp x and y variables so we dont have to type out so much

#ifdef TEST_LIVE_VIDEO
    //del_x = theObject[0]-x;
    //del_y = theObject[1]-y;
    x = theObject[0];
    y = theObject[1];

#else
    //del_x = (theObject[0]+520)-x;
    //del_y = (theObject[1]+90)-y;
    x = theObject[0]+520;
    y = theObject[1]+90;

#endif
    //leastSquare lsf(roi);
    point_input.x = x;
    point_input.y = y;
    lsf.addPoint(point_input);
    if(lsf.getSize()==8) {
	if(lsf.testMonotonic()==1) {
	    //cout << "test point" << endl;
	    lsf.computeLSF();
	    int num_rebounds = lsf.computeRebounds();
	    //if(num_rebounds==0) {
		//cv::Point2i final_p = leastSquare::noRebound();
		//line(cameraFeed,Point(x,y),Point(final_p.x,final_p.y),Scalar(0,0,255),2);
	    //}
	    //cout << num_rebounds << endl;
	    cv::Point2i rebound_p[num_rebounds];
	    lsf.getRebound(&rebound_p[num_rebounds], num_rebounds);
	    line(cameraFeed,Point(x,y),Point(rebound_p[0].x,rebound_p[0].y),Scalar(0,0,255),2);
	    //cout << rebound_p[0].x << " " << rebound_p[0].y << endl;
	    if(num_rebounds>=1) 
		line(cameraFeed,Point(rebound_p[0].x,rebound_p[0].y),Point(rebound_p[1].x,rebound_p[1].y),Scalar(0,0,255),2);
		
	    if(num_rebounds>=2)
		line(cameraFeed,Point(rebound_p[1].x,rebound_p[1].y),Point(rebound_p[2].x,rebound_p[2].y),Scalar(0,0,255),2);

	    if(num_rebounds>=3)
		line(cameraFeed,Point(rebound_p[2].x,rebound_p[2].y),Point(rebound_p[3].x,rebound_p[3].y),Scalar(0,0,255),2);

	    if(num_rebounds>=4)
		line(cameraFeed,Point(rebound_p[3].x,rebound_p[3].y),Point(rebound_p[4].x,rebound_p[4].y),Scalar(0,0,255),2);
	    
	}

    }
    
    // Print the object position
    //cout << xpos << " " << ypos << endl;
}
//-----------------------------------------------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------------------------------------------
int main() {
    
    // OpenCV frame matrices
    cv::Mat frame0, frame0_warped, frame1, frame1_warped, result;

    cv::cuda::GpuMat gpu_frame0, gpu_frame0_warped, gpu_frame1, gpu_frame1_warped, gpu_grayImage0, gpu_grayImage1, gpu_differenceImage, gpu_thresholdImage;

    int toggle, frame_count;
    //Matrix to store each frame of the webcam feed
    Mat cameraFeed;
    //matrix storage for HSV images
    Mat HSV0;
    Mat HSV1;
    Mat HSV2;
    Mat HSV3;
    //matrix storage for binary threshold image
    Mat threshold0;
    Mat threshold1;
    Mat threshold2;
    //x and y values for the location of the object
    int x=0, y=0;
    //define regions of interest for bottom and top paddles
    cv::Rect roi_bot(520, 625, 308, 30);
    cv::Rect roi_top(520, 60, 308, 30);
    //create slider bars for HSV filtering
    //createTrackbars();
    //video capture object to acquire webcam feed
#ifdef TEST_LIVE_VIDEO

    // Camera video pipeline
    std::string pipeline = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

#else

    // MP4 file pipeline
    std::string pipeline = "filesrc location=/home/nvidia/Videos/pong_video.mp4 ! qtdemux name=demux ! h264parse ! omxh264dec ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

#endif
    std::cout << "Using pipeline: " << pipeline << std::endl;
 
    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cout << "Connection failed" << std::endl;
        return -1;
    }   
    // Input Quadrilateral or Image plane coordinates
    Point2f inputQuad[4];
    // Output Quadrilateral or World plane coordinates
    Point2f outputQuad[4];
    // Lambda Matrix
    cv::Mat lambda( 2, 4, CV_32FC1 );
    // The 4 points that define quadrilateral, from top-left going clockwise
    // These points are the sides of the rect box used as input

    // OpenCV coordinate system is based on rows, columns
    inputQuad[0] = Point2f(520, 80);
    inputQuad[1] = Point2f(880, 77);
    inputQuad[2] = Point2f(923, 672);
    inputQuad[3] = Point2f(472, 655);
    // The 4 points where mapping is to be done, clockwise from top-left
    outputQuad[0] = Point2f(437, 0);
    outputQuad[1] = Point2f(842, 0);
    outputQuad[2] = Point2f(842, 719);
    outputQuad[3] = Point2f(437, 719);

    // Get the perspective transform matrix lambda
    lambda = cv::getPerspectiveTransform(inputQuad,outputQuad);

    // Capture the first frame with GStreamer
    cap >> frame0;
    gpu_frame0.upload(frame0);

    // Warp Perspective
    cv::cuda::warpPerspective(gpu_frame0,gpu_frame0_warped,lambda,gpu_frame0.size());
    gpu_frame0_warped.download(frame0_warped);

    // Convert the frames to gray scale (monochrome)
    cv::cuda::cvtColor(gpu_frame0,gpu_grayImage0,cv::COLOR_BGR2GRAY);

    // Initialize 
    toggle = 0;
    frame_count = 0;

    while (1) {
        if (toggle == 0) {
           // Get a new frame from file
           cap >> frame1;
	   // Upload to GPU memory
	   gpu_frame1.upload(frame1);
	   // Warp Perspective
	   cv::cuda::warpPerspective(gpu_frame1,gpu_frame1_warped,lambda,gpu_frame1.size());
	   gpu_frame1_warped.download(frame1_warped);
           // Convert the frames to gray scale (monochrome)    
           cv::cuda::cvtColor(gpu_frame1_warped,gpu_grayImage1,cv::COLOR_BGR2GRAY);
           toggle = 1;
        } 
        else {
	   // Get a new video frame
	   cap >> frame0;
	   // Upload to GPU memory
           gpu_frame0.upload(frame0);
	   // Warp Perspective
	   cv::cuda::warpPerspective(gpu_frame0,gpu_frame0_warped,lambda,gpu_frame0.size());
	   gpu_frame0_warped.download(frame0_warped);
     	   // Convert to grayscale
           cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);
           toggle = 0;
	}
 
	// Compute the absolte value of the difference
	cv::cuda::absdiff(gpu_grayImage0, gpu_grayImage1, gpu_differenceImage);


	// Threshold the difference image
        cv::cuda::threshold(gpu_differenceImage, gpu_thresholdImage, 50, 255, cv::THRESH_BINARY);

        gpu_thresholdImage.download(result);

	// Find the location of any moving object and show the final frame
	if (toggle == 0) {
		//searchForMovement(thresholdImage,frame0);
                
		//cv::line(frame0,inputQuad[0],inputQuad[1],Scalar(0,255,0),2);
		//cv::line(frame0,inputQuad[1],inputQuad[2],Scalar(0,255,0),2);
		//cv::line(frame0,inputQuad[2],inputQuad[3],Scalar(0,255,0),2);
		//cv::line(frame0,inputQuad[3],inputQuad[0],Scalar(0,255,0),2);
		
		//convert frame from BGR to HSV colorspace
		cvtColor(frame0_warped,HSV0,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV0,Scalar(H_MIN_bot,S_MIN_bot,V_MIN_bot),Scalar(H_MAX_bot,S_MAX_bot,V_MAX_bot),threshold0);
		searchForPaddle(threshold0,frame0_warped,bot_roi);
		
		inRange(HSV0,Scalar(H_MIN_top,S_MIN_top,V_MIN_top),Scalar(H_MAX_top,S_MAX_top,V_MAX_top),threshold1);
		searchForPaddle(threshold1,frame0_warped,top_roi);

		searchForMovement(result,frame0_warped);
	        imshow("Frame", frame0_warped);
		//imshow(windowName2,threshold);
		//imshow(windowName1,HSV0);

		//find the corners of the playing field
		cvtColor(frame0,HSV2,COLOR_BGR2HSV);
		inRange(HSV2,Scalar(H_MIN_fld,S_MIN_fld,V_MIN_fld),Scalar(H_MAX_fld,S_MAX_fld,V_MAX_fld),threshold2);
		findCorners(threshold2, frame0);
		//imshow("Original", frame0);
	}
	else {
		//searchForMovement(thresholdImage,frame1);

		//cv::line(frame0,inputQuad[0],inputQuad[1],Scalar(0,255,0),2);
		//cv::line(frame0,inputQuad[1],inputQuad[2],Scalar(0,255,0),2);
		//cv::line(frame0,inputQuad[2],inputQuad[3],Scalar(0,255,0),2);
		//cv::line(frame0,inputQuad[3],inputQuad[0],Scalar(0,255,0),2);
		
		//convert frame from BGR to HSV colorspace
		cvtColor(frame1_warped,HSV1,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV1,Scalar(H_MIN_bot,S_MIN_bot,V_MIN_bot),Scalar(H_MAX_bot,S_MAX_bot,V_MAX_bot),threshold0);
		searchForPaddle(threshold0,frame1_warped,bot_roi);

		inRange(HSV1,Scalar(H_MIN_top,S_MIN_top,V_MIN_top),Scalar(H_MAX_top,S_MAX_top,V_MAX_top),threshold1);
		searchForPaddle(threshold1,frame1_warped,top_roi);

		searchForMovement(result,frame1_warped);
	        imshow("Frame", frame1_warped);
		//imshow(windowName2,threshold);
		//imshow(windowName1,HSV1);

		//find the corners of the playing field
		cvtColor(frame1,HSV3,COLOR_BGR2HSV);
		inRange(HSV3,Scalar(H_MIN_fld,S_MIN_fld,V_MIN_fld),Scalar(H_MAX_fld,S_MAX_fld,V_MAX_fld),threshold2);
		findCorners(threshold2, frame1);
		//imshow("Original", frame1);
	}
	frame_count++;
        cv::waitKey(1); //needed to show frame
    }
}
