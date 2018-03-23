// lestSquare.h: least square fit header file
// by Jordan Spangler
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <algorithm>

#define FALSE 0
#define TRUE 1

class leastSquare {
	private:
		cv::Point2i m_array[8];
		cv::Point2i m_rebounds;
		unsigned int m_size;
		unsigned int m_index;
		cv::Rect m_roi;
		double m_x_slope;
		double m_y_slope;
		double x_intercept;
		double y_intercept;
		float t_avg;
	public:
		leastSquare(cv::Rect roi);
		unsigned int getSize();
		void addPoint(cv::Point2i point);
		int testMonotonic();
		void computeLSF();
		int computeRebounds();
		void getRebound(cv::Point2i *ptr,int n);
		//cv::Point2i noRebound();
};
