#include "leastSquare.h"
#include <iostream>
//using std::cout;
//using std::endl;

leastSquare::leastSquare(cv::Rect roi) {
	//initialize all private variables
	cv::Point2i m_array[8];
	cv::Point2i m_rebounds;
	unsigned int m_size;
	unsigned int m_index = 0;
	cv::Rect m_roi=roi;
	double m_x_slope;
	double m_y_slope;
	double x_intercept;
	double y_intercept;
	float t_avg = 4.5;
}
unsigned int leastSquare::getSize() {
	m_size = sizeof(m_array)/sizeof(m_array[0]);
	return m_size;
}
void leastSquare::addPoint(cv::Point2i point) {
	m_array[7] = m_array[6];
	m_array[6] = m_array[5];
	m_array[5] = m_array[4];
	m_array[4] = m_array[3];
	m_array[3] = m_array[2];
	m_array[2] = m_array[1];
	m_array[1] = m_array[0];
	m_array[0] = point;
	//std::cout << m_array[1] << std::endl;
}
int leastSquare::testMonotonic() { //this section needs work, does not account for all monotonic cases
	
	int i;
	int m_index1 = 0;
	int m_index2 = 0;
	int m_index3 = 0;
	int m_index4 = 0;
	for(i=0;i<8;) {
		//std::cout << m_array[i].x << std::endl;
		if((m_array[i].x>m_array[i+1].x) && (m_array[i].y>m_array[i+1].y)) {
			m_index1++;
		}
		else if((m_array[i].x<m_array[i+1].x) && (m_array[i].y>m_array[i+1].y)) {
			m_index2++;
		}
		else if((m_array[i].x<m_array[i+1].x) && (m_array[i].y<m_array[i+1].y)) {
			m_index3++;
		}
		else if((m_array[i].x>m_array[i+1].x) && (m_array[i].y<m_array[i+1].y)) {
			m_index4++;
		}
		i++;
	}
	//std::cout << m_index1 << " " << m_index2 << " " << m_index3 << " " << m_index4 << std::endl;
	if(m_index1==7)
		return 1;
	else if(m_index2==7)
		return 1;
	else if(m_index3==7)
		return 1;
	else if(m_index4==7)
		return 1;
	else
		return 0;
}
void leastSquare::computeLSF() {

	// calculate slope for x points
	int sum_x = 0;
	for (int i = 0; i < 8; i++) {
		sum_x = sum_x + m_array[i].x;
	}
	float x_avg = sum_x/8;
	m_x_slope = ((m_array[0].x-x_avg)*(-3.5)+(m_array[1].x-x_avg)*(-2.5)+(m_array[2].x-x_avg)*(-1.5)+(m_array[3].x-x_avg)*(-0.5)+(m_array[4].x-x_avg)*(0.5)+(m_array[5].x-x_avg)*(1.5)+(m_array[6].x-x_avg)*(2.5)+(m_array[7].x-x_avg)*(3.5))/42;
	// calculate the x-intercept
	x_intercept = x_avg-m_x_slope*t_avg;
	
	// calculate slope for y points
	int sum_y = 0;
	for (int i = 0; i < 8; i++) {
		sum_y = sum_y + m_array[i].y;
	}
	float y_avg = sum_y/8;
	m_y_slope = ((m_array[0].y-y_avg)*(-3.5)+(m_array[1].y-y_avg)*(-2.5)+(m_array[2].y-y_avg)*(-1.5)+(m_array[3].y-y_avg)*(-0.5)+(m_array[4].y-y_avg)*(0.5)+(m_array[5].y-y_avg)*(1.5)+(m_array[6].y-y_avg)*(2.5)+(m_array[7].y-y_avg)*(3.5))/42;

	// calculate the y-intercept
	y_intercept = y_avg-m_y_slope*t_avg;

	t_avg++; //increment the mean, since the mean of a rolling time vector increases by 1 each iteration	
	//std::cout << "rise" << std::endl;	
	//std::cout << m_y_slope << std::endl;
	//std::cout << "run" << std::endl;
	//std::cout << m_x_slope << std::endl;
	//std::cout << "y-intercept" << std::endl;	
	//std::cout << y_intercept << std::endl;
	//std::cout << "x-intercept" << std::endl;
	//std::cout << x_intercept << std::endl;

}
int leastSquare::computeRebounds() {
	if((m_array[0].y>91) && (m_array[0].y<624)) {
		float t = t_avg;
		double x_int = x_intercept;
		double y_int = y_intercept;
		double m_x_slp = m_x_slope;
		double x_rebound;
		double y_rebound;
		double t_x_lim;
		double t_y_lim;
		for(int j=0; j<5; j++) {  //increments counter for every rebound point found, outputs counter
			//
			//compute time until x boundary
			if(m_x_slope>0)
				t_x_lim = (x_int - 828)/m_x_slp;
	
			else
				t_x_lim = (x_int - 520 )/m_x_slp;

			//compute time until y boundary
			if(m_y_slope>0)
				t_y_lim = (y_int - 625)/m_y_slope;
	
			else
				t_y_lim = (y_int - 90)/m_y_slope;

			
			//compute how many wall rebounds will occur
			if(t_y_lim<=t_x_lim)
				return j;
			else {
				x_rebound = m_x_slp*t_x_lim + m_array[0].x;
				y_rebound = m_y_slope*t_x_lim + m_array[0].y;
				m_x_slp = -m_x_slp; //reverse slope for bounce
				t = t+t_x_lim; //project time at bounce
				x_int = x_rebound-m_x_slp*t; // re-calculate the x-intercept
				y_int = y_rebound-m_y_slope*t; // re-calculate the y-intercept
			}
			
		}
	}
	
}
void leastSquare::getRebound(cv::Point2i *ptr, int n) {
	float t = t_avg;
	int i = n;
	double x_int = x_intercept;
	double y_int = y_intercept;
	double m_x_slp = m_x_slope;
	double t_x_lim;
	double t_y_lim;
	for(int k=0;k<=i;k++) {
		//compute time until x boundary
		if(m_x_slp>0)
			t_x_lim = (x_int - 828)/m_x_slp;
		else if(m_x_slp<0)
			t_x_lim = (x_int - 520 )/m_x_slp;
		//compute time until y boundary
		if(m_y_slope>0)
			t_y_lim = (y_int - 625)/m_y_slope;
		else
			t_y_lim = (y_int - 90)/m_y_slope;
		//find final point where ball crosses horizontal boundary
		if(t_y_lim<=t_x_lim) {
			m_rebounds.x = m_x_slp*t_y_lim + m_array[0].x; //was x_int
			m_rebounds.y = m_y_slope*t_y_lim + m_array[0].y; // was y_int
			*ptr = m_rebounds;
			ptr++;
			//size of m_rebounds will be 1 larger than number of rebounds
		}	
		else {	
			m_rebounds.x = m_x_slp*t_x_lim + m_array[0].x;
			m_rebounds.y = m_y_slope*t_x_lim + m_array[0].y;
			*ptr = m_rebounds;
			m_x_slp = -m_x_slp; //reverse slope for bounce
			t = t+t_x_lim; //project time at bounce
			x_int = m_rebounds.x-m_x_slp*t; // re-calculate the x-intercept
			y_int = m_rebounds.y-m_y_slope*t; // re-calculate the y-intercept
		}
		std::cout << m_rebounds << std::endl;
	}
}
//cv::Point2i leastSquare::noRebound() {
	//double t_y_lim;
	//compute time until y boundary
	//if(m_y_slope>0)
		//t_y_lim = (625 - y_int)/m_y_slope;
	
	//else
		//t_y_lim = (90 - y_int)/m_y_slope;

	//m_rebounds.x[0] = m_x_slope*t_y_lim + x_intercept;
	//m_rebounds.y[0] = m_y_slope*t_y_lim + y_intercept;
	//return m_rebounds;
//}
