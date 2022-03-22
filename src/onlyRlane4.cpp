#include <ros/ros.h>
#include "platooning_perception/multican.h"
#include "platooning_perception/lane.h"

#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt32.h>

using namespace std;
using namespace cv;

class LaneDetector {
private:
	cv_bridge::CvImagePtr cv_ptr_;

	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Subscriber multican_sub_;
	ros::Subscriber time_sub_;
	ros::Subscriber target_sub_;
	
	ros::Publisher image_pub_;
	ros::Publisher range_pub_;

	platooning_perception::lane lane_msg_;	

	stringstream ss_;
	int get_time_sec_;
	int get_time_nsec_;

	unsigned int target_id_;
	float distance_;
	int check_recv_;

	// thresholds
	int sobel_min_th_;
	int sobel_max_th_;
	int hls_min_th_;
	int hls_max_th_;
	
	bool check_;
	int last_leftx_base_;
	int last_rightx_base_;

	Point2f src_[4];
	Point2f dst_[4];
	
	vector<int> left_x_;
	vector<int> left_y_;
	vector<int> right_x_;
	vector<int> right_y_;
	vector<int> left_lane_inds_;
	vector<int> right_lane_inds_;
	
	Mat left_coef_;
	Mat right_coef_;

	float center_position_;
	float left_curve_radius_;
	float right_curve_radius_;

	Mat frame_;
	Mat resized_frame_;
	Mat unwarp_frame_;
	Mat binary_frame_;
	Mat result_frame_;
	Mat Minv_;

	VideoCapture cap_;

public:
	LaneDetector(void) {
		initSetup();
	}

	~LaneDetector(void) {
		left_x_.clear();
		left_y_.clear();
		right_x_.clear();
		right_y_.clear();
		left_lane_inds_.clear();
		right_lane_inds_.clear();
	}

	void initSetup(void) {
		//cap_.open("/home/woonghyun/testroad4.mp4");
	
		multican_sub_ = nh_.subscribe("can_msgs", 10, &LaneDetector::MultiCanCallback, this);
		//image_sub_ = nh_.subscribe("usb_cam/image_raw", 10, &LaneDetector::ImageCallback, this);
		image_sub_ = nh_.subscribe("/videofile/image_raw", 10, &LaneDetector::ImageCallback, this);
		time_sub_ = nh_.subscribe("/videofile/camera_info", 10, &LaneDetector::CameraInfoCallback, this);
		target_sub_ = nh_.subscribe("target", 10, &LaneDetector::TargetCallback, this);
		
		image_pub_ = nh_.advertise<sensor_msgs::Image>("lane_image", 10);
		range_pub_ = nh_.advertise<platooning_perception::lane>("lane_range", 10);

		sobel_min_th_ = 10;
		sobel_max_th_ = 255;
		hls_min_th_ = 220;
		hls_max_th_ = 255;
	
		check_ = true;
		last_leftx_base_ = 0;
		last_rightx_base_ = 0;
	}

	void ImageCallback(const::sensor_msgs::Image::ConstPtr &image_msg) {
		try {
		    	printf("ImageCB\n");
			cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
			cv::resize(cv_ptr_->image,cv_ptr_->image, Size(1280,720),CV_INTER_LINEAR);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		        return;
		}
		frame_ = cv_ptr_->image;
	}

	void CameraInfoCallback(const::sensor_msgs::CameraInfo::ConstPtr &camera_info_msg) {
		get_time_sec_ = camera_info_msg->header.stamp.sec;
		get_time_nsec_ = camera_info_msg->header.stamp.nsec;
	}

	void TargetCallback(const::std_msgs::UInt32::ConstPtr &target_msg) {
		distance_ = target_msg->data;
		check_recv_ = 1;		
	}

	void MultiCanCallback(const::platooning_perception::multican::ConstPtr &multican_msg) {
		platooning_perception::multican can_data = *multican_msg;

		for(int i=0;i<can_data.can_msgs.size();i++) {
			if(target_id_ == can_data.can_msgs[i].id)
				distance_ = (multican_msg->can_msgs[i].data[2] & 0x7F) + multican_msg->can_msgs[i].data[1];		
		}			
	}
	
	void unwarp(Mat frame) {
		Mat M = getPerspectiveTransform(src_, dst_);
		Minv_ = getPerspectiveTransform(dst_, src_);
		Mat warped(frame.rows, frame.cols, frame.type());
		warpPerspective(frame, warped, M, warped.size(), INTER_LINEAR, BORDER_CONSTANT);
		unwarp_frame_ = warped;
	}

	Mat abs_sobel_thresh(Mat frame) {
		Mat gray_frame, sobel_frame, abs_sobel_frame;
		double min, max;

		cvtColor(frame, gray_frame, CV_BGR2GRAY);
		Sobel(gray_frame, sobel_frame, CV_64F, 1, 0);
		abs_sobel_frame = abs(sobel_frame);

		minMaxLoc(abs_sobel_frame, &min, &max);
		
		Mat scaled_sobel_binary_frame = Mat::zeros(sobel_frame.rows, sobel_frame.cols, CV_8UC1);
		
		for(int j=0;j<sobel_frame.rows;j++) {
			for(int i=0;i<sobel_frame.cols;i++) {
				double val = abs_sobel_frame.at<uchar>(j, i) * 255 / max;
				if(val >= sobel_min_th_ && val <= sobel_max_th_) scaled_sobel_binary_frame.at<uchar>(j, i) = 255;	
			}
		}
		return scaled_sobel_binary_frame;
	}

	Mat hls_lthresh(Mat frame) {
		Mat hls_frame;
		vector<Mat> hls_images(3);
		double min, max;

		cvtColor(frame, hls_frame, CV_BGR2HLS);
		split(hls_frame, hls_images);
		
		minMaxLoc(hls_images[1], &min, &max);

		for(int j=0;j<hls_images[1].rows;j++) {
			for(int i=0;i<hls_images[1].cols;i++) {
				if(hls_images[1].at<uchar>(j, i) * 255 / max > hls_min_th_ && hls_images[1].at<uchar>(j, i) * 255 / max <= hls_max_th_) hls_images[1].at<uchar>(j, i) = 255;
			}
		}

		return hls_images[1];
	}

	void pipeline(Mat frame) {
		Mat abs_sobel_frame = abs_sobel_thresh(frame);
		Mat l_frame = hls_lthresh(frame);
		Mat combined_frame = Mat::zeros(frame.rows, frame.cols, CV_8UC1);

		for(int j=0;j<combined_frame.rows;j++) {
			for(int i=0;i<combined_frame.cols;i++) {
				if(abs_sobel_frame.at<uchar>(j, i) == 255 || l_frame.at<uchar>(j, i) == 255) combined_frame.at<uchar>(j, i) = 255;
			}
		}
		binary_frame_ = combined_frame;
	}

	double gaussian(double x, double mu, double sig) {
		return exp((-1) * pow(x - mu, 2.0) / (2 * pow(sig, 2.0)));
	}

	void roi_range(float distance) {
		int roi_y, roi_lx, roi_rx;

		if(distance < 80) {
			roi_y = 80;
			// ROS_INFO("ROI CASE 1.");
		}
		else {
			distance = (distance / 10 - 1) * 10 + 5;
			roi_y = distance;	
			// ROS_INFO("ROI CASE 2.");
		}
		roi_lx = ((700 - roi_y) * 0.6) + 200;
	       	roi_rx = ((roi_y - 700) * 0.6) + 980;
	
		src_[0] = Point2f(roi_lx, roi_y);
		src_[1] = Point2f(roi_rx, roi_y);	
	}

	int arrMaxIdx(int hist[], int cols, int start, int end) {
		int max_index = -1;
		int max_val = 0;

		for(int i=start;i<end;i++) {
			if(max_val < hist[i]) {
				max_val = hist[i];
				max_index = i;
			}
		}
		return max_index;
	}	

	void sliding_window_polyfit(Mat frame) {
		int hist[frame.cols] = {0, };
		vector<int> nonzero_x;
		vector<int> nonzero_y;
		vector<int> good_left_inds;
		vector<int> good_right_inds;

		for(int j=0;j<frame.rows;j++) {
			for(int i=0;i<frame.cols;i++) {		
				if(frame.at<uchar>(j, i) == 255) {
					hist[i] += 1;
					nonzero_x.push_back(i);
					nonzero_y.push_back(j);
				}
			}
		}

		if(last_leftx_base_ != 0 || last_rightx_base_ != 0) {
			double weight_distrib[frame.cols] = {0, };

			int distrib_width = 200;
			double sigma = distrib_width / 12.0;
			
			int leftx_start = last_leftx_base_ - distrib_width / 2;
			int leftx_end = last_leftx_base_ + distrib_width / 2;
			
			int rightx_start = last_rightx_base_ - distrib_width / 2;
			int rightx_end = last_rightx_base_ + distrib_width / 2;

			for(int i=0;i<frame.cols;i++) {
				if(i >= leftx_start && i <= leftx_end) {
					weight_distrib[i] = gaussian(i, last_leftx_base_, sigma);
					hist[i] *= weight_distrib[i];
				}
				else if(i >= rightx_start && i <= rightx_end) {
					weight_distrib[i] = gaussian(i, last_rightx_base_, sigma);
					hist[i] *= weight_distrib[i];
				}
			}
		}

		int mid_point = frame.cols / 2;
		int quarter_point = mid_point / 2;
			
		int left_offset = 80;
		int nwindows = 10;
		int window_height = frame.rows / nwindows;
		int margin = 50;
		int min_pix = 400;
		int good_pix = 1100;
		int max_pix = 3100;
		int dist = 890;
		int count;

		int leftx_base = arrMaxIdx(hist, frame.cols, quarter_point - 3 * left_offset, quarter_point - left_offset) + quarter_point - 3 * left_offset; 
		int rightx_base = arrMaxIdx(hist, frame.cols, mid_point + quarter_point, mid_point + quarter_point + 3 * left_offset) + mid_point + quarter_point; 
		
		int leftx_current = leftx_base;
		int rightx_current = rightx_base;

		last_leftx_base_ = leftx_base;
		last_rightx_base_ = rightx_base;

		ROS_INFO("LEFT: %d", leftx_base);
		ROS_INFO("RIGHT: %d", rightx_base);

		if(check_) count = 7;
		else count = 0;

		for(int i=0;i<nwindows;i++) {
			int win_y_low = frame.rows - (i + 1) * window_height;
			int win_y_high = frame.rows - i * window_height;

			int win_xleft_low = leftx_current - margin;
			int win_xleft_high = leftx_current - margin;

			int win_xright_low = rightx_current - margin;
			int win_xright_high = rightx_current + margin;

			for(int i=0;i<nonzero_x.size();i++) {
				if(nonzero_y[i] >= win_y_low && nonzero_y[i] < win_y_high && nonzero_x[i] >= win_xleft_low && nonzero_x[i] < win_xleft_high) good_left_inds.push_back(i);
				if(nonzero_y[i] >= win_y_low && nonzero_y[i] < win_y_high && nonzero_x[i] >= win_xright_low && nonzero_x[i] < win_xright_high) good_right_inds.push_back(i);
			}
			
			if(good_left_inds.size() > min_pix) {
				int sum = 0;
				int size = good_left_inds.size();
				for(int i=0;i<size;i++) {
					sum += nonzero_x[i];
				}
				leftx_current = sum / size;
				if(max_pix > size && size > good_pix) count += 1;
			}
			if(good_right_inds.size() > min_pix) {
				int sum = 0;
				int size = good_right_inds.size();
				for(int i=0;i<size;i++) {
					sum += nonzero_x[i];
				}
				rightx_current = sum / size;
			}		
			left_lane_inds_.insert(left_lane_inds_.end(), good_left_inds.begin(), good_left_inds.end());	
			right_lane_inds_.insert(right_lane_inds_.end(), good_right_inds.begin(), good_right_inds.end());
		}
	

		for(int i=0;i<left_lane_inds_.size();i++) {
			left_x_.push_back(nonzero_x[i]);
			left_y_.push_back(nonzero_y[i]);
		}
		for(int i=0;i<right_lane_inds_.size();i++) {
			right_x_.push_back(nonzero_x[i]);
			right_y_.push_back(nonzero_y[i]);
		}

		nonzero_x.clear();
		nonzero_y.clear();
		good_left_inds.clear();
		good_right_inds.clear();
		

		if(left_x_.size() != 0) {
			if(count > 4) {
				left_coef_ = polyfit(left_x_, left_y_);	
				count = 0;
			}
			else {
				for(int i=0;i<left_x_.size();i++) left_x_[i] -= dist;
				left_coef_ = polyfit(left_x_, left_y_);
				count = 0;
			}
		}
		if(right_x_.size() != 0) {
			right_coef_ = polyfit(right_x_, right_y_);
		}
	}	
	
	Mat polyfit(vector<int> x_val, vector<int> y_val) {
		Mat coef(3, 1, CV_32F);
		int i,j,k;
		int N = x_val.size();
		int n = 2;
		float x[N], y[N];
		for(int q=0;q<N;q++) {
			x[q] = x_val[q];
			y[q] = y_val[q];
		}
		float X[2*n+1];                        
	    	for (i=0;i<2*n+1;i++)
	    	{
			X[i]=0;
			for (j=0;j<N;j++)
		    		X[i]=X[i]+pow(x[j],i);        
	    	}
	    	float B[n+1][n+2],a[n+1];            
	    	for (i=0;i<=n;i++)
			for (j=0;j<=n;j++)
		    		B[i][j]=X[i+j];           
	    	float Y[n+1];                    
	    	for (i=0;i<n+1;i++)
	    	{    
			Y[i]=0;
			for (j=0;j<N;j++)
			Y[i]=Y[i]+pow(x[j],i)*y[j];        
	    	}
	    	for (i=0;i<=n;i++)
			B[i][n+1]=Y[i];                
	    	n=n+1;               
	    	for (i=0;i<n;i++)                    
			for (k=i+1;k<n;k++)
		    		if (B[i][i]<B[k][i])
		        		for (j=0;j<=n;j++)
		        		{
		            			float temp=B[i][j];
		            			B[i][j]=B[k][j];
		           			B[k][j]=temp;
		       			 }	
	    
	    	for (i=0;i<n-1;i++)           
			for (k=i+1;k<n;k++)
		   	{
		       		 float t=B[k][i]/B[i][i];
		      		 for (j=0;j<=n;j++)
		           		 B[k][j]=B[k][j]-t*B[i][j];    
		    	}
	    	for (i=n-1;i>=0;i--)               
	    	{                        
			a[i]=B[i][n];                
			for (j=0;j<n;j++)
		    		if (j!=i)
		        		a[i]=a[i]-B[i][j]*a[j];
			a[i]=a[i]/B[i][i];
			coef.at<float>(i, 0) = a[i];
	    	}
		return coef;
	}

	void calc_curv_rad_and_center_dist(Mat frame, Mat l_fit, Mat r_fit, vector<int> lx, vector<int> ly, vector<int> rx, vector<int> ry) {
		int car_position = frame.cols / 2;
		int lane_center_position;
		float center_position;
		float left_cr;
		float right_cr;
		
		float ym_per_pix = 3.048 / 100.0;
		float xm_per_pix = 3.7 / 378.0;

		Mat left_coef_cr(3, 1, CV_32F);
		Mat right_coef_cr(3, 1, CV_32F);

		if(lx.size() != 0 && rx.size() != 0) {
			for(int i=0;i<lx.size();i++) {
				lx[i] = lx[i] * xm_per_pix;
				ly[i] = ly[i] * ym_per_pix;
			}
			for(int i=0;i<rx.size();i++) {
				rx[i] = rx[i] * xm_per_pix;
				ry[i] = ry[i] * ym_per_pix;
			}

			left_coef_cr = polyfit(lx, ly);
			right_coef_cr = polyfit(rx, ry);

			left_cr = powf((1 + powf(2 * left_coef_cr.at<float>(2, 0) * 0 * xm_per_pix + left_coef_cr.at<float>(1, 0), 2)), 1.5) / fabs(2 * left_coef_cr.at<float>(2, 0) + 0.000001);
			right_cr = powf((1 + powf(2 * right_coef_cr.at<float>(2, 0) * 0 * xm_per_pix + right_coef_cr.at<float>(1, 0), 2)), 1.5) / fabs(2 * right_coef_cr.at<float>(2, 0) + 0.000001);
			
			left_curve_radius_ = left_cr;
			right_curve_radius_ = right_cr;
		}

		if(!l_fit.empty() && !r_fit.empty()) {
			lane_center_position = (l_fit.at<float>(0, 0) + r_fit.at<float>(0, 0)) / 2;
	       	
			center_position = (car_position - lane_center_position) * ym_per_pix;

			center_position_ = center_position;
		}
	}
	
	Mat draw_lane(Mat frame, Mat warped_frame, Mat left_coef, Mat right_coef, Mat Minv) {
		Mat new_frame = frame.clone();
	
		int height = warped_frame.rows;
		int width = warped_frame.cols;
		
		vector<Point2f> left_point;
		vector<Point2f> right_point;

		vector<Point2f> warped_left_point;
		vector<Point2f> warped_right_point;

		vector<Point> left_points;
		vector<Point> right_points;
		
		for(int i=0;i<height;i++) {
			Point2f temp_left_point;
			Point2f temp_right_point;
			temp_left_point.x = left_coef.at<float>(2, 0) * pow(i, 2) + left_coef.at<float>(1, 0) * i + left_coef.at<float>(0, 0);
			temp_left_point.y = i;
			temp_right_point.x = right_coef.at<float>(2, 0) * pow(i, 2) + right_coef.at<float>(1, 0) * i + right_coef.at<float>(0, 0);
			temp_right_point.y = i;
			left_point.push_back(temp_left_point);
			right_point.push_back(temp_right_point);
		}

		perspectiveTransform(left_point, warped_left_point, Minv);	
		perspectiveTransform(right_point, warped_left_point, Minv);	
		
		for(int i=0;i<height;i++) {
			Point temp_left_point;
			Point temp_right_point;
			
			lane_msg_.left.x[i] = warped_left_point[i].x;
			lane_msg_.left.y[i] = warped_left_point[i].y;
			lane_msg_.right.x[i] = warped_right_point[i].x;
			lane_msg_.right.y[i] = warped_right_point[i].y;
			
			temp_left_point.x = (int)warped_left_point[i].x;
			temp_left_point.y = (int)warped_left_point[i].y;
			temp_right_point.x = (int)warped_right_point[i].x;
			temp_right_point.y = (int)warped_right_point[i].y;
			
			left_points.push_back(temp_left_point);
			right_points.push_back(temp_right_point);
		}
	
		const Point *left_points_point = (const cv::Point*) Mat(left_points).data;	
		int left_points_number = Mat(left_points).rows;
		const Point *right_points_point = (const cv::Point*) Mat(right_points).data;
		int right_points_number = Mat(right_points).rows;
		
		polylines(new_frame, &left_points_point, &left_points_number, 1, false, Scalar(0, 255, 0), 30);	
		polylines(new_frame, &left_points_point, &left_points_number, 1, false, Scalar(0, 255, 0), 30);

		return new_frame;
	}
	
	void run(void) {
		cap_ >> frame_;	
		
		resize(frame_, resized_frame_, Size(1280, 720));

		int height = frame_.rows; 
		int width = frame_.cols;
		
		roi_range(400);

		src_[2] = Point2f(200, 700);
		src_[3] = Point2f(980, 700);

		dst_[0] = Point2f(250, 0);
		dst_[1] = Point2f(width - 250, 0);
		dst_[2] = Point2f(250, height);
		dst_[3] = Point2f(width - 250, height);

		unwarp(frame_);

		pipeline(unwarp_frame_);
		
		sliding_window_polyfit(binary_frame_);

		calc_curv_rad_and_center_dist(binary_frame_, left_coef_, right_coef_, left_x_, left_y_, right_x_, right_y_);
		
		result_frame_ = draw_lane(frame_, unwarp_frame_, left_coef_, right_coef_, Minv_);
	
		range_pub_.publish(lane_msg_);
			
		// check_ = false;

		imshow("FRAME", resized_frame_);
		imshow("UNWARP FRAME", unwarp_frame_); 
		imshow("BINARY FRAME", binary_frame_);
		imshow("RESULT FRAME", result_frame_);
	
		waitKey(1);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "platooning_perceptionection_node");

	//ros::Rate loop_rate(10);

	LaneDetector ld;

	while(ros::ok()) {
		ld.run();
		ros::spin();
	//	loop_rate.sleep();
	}

	return 0;
}
