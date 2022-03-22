#include <ros/ros.h>
#include <iostream>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <cmath>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <platooning_perception/can.h>
#include <platooning_perception/multican.h>

#include "platooning_perception/kalman2.hpp"
//#include "platooning_perception/AdaptiveGating.hpp"

using namespace std;


namespace radar_kalman{

class RadarKalman
{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

	visualization_msgs::Marker mk_radar;
	visualization_msgs::MarkerArray ma;

	vector<float> rx_float;
	vector<float> ry_float;

	array<float, 16> radar_x;
	array<float, 16> radar_y;

	float radar_pos_x = 0;
	float radar_pos_y = 0;

public:
	RadarKalman(ros::NodeHandle node)
			:n(node)
	{
		ROS_INFO("Start!");
		sub = n.subscribe("/filtered_msgs",100, &RadarKalman::Callback, this);
		pub = n.advertise<visualization_msgs::MarkerArray>("/radar_kalman", 10);

	}

	~RadarKalman()
	{
	}

	void initMarker()
	{

			//-------------------------------------------------radar
			mk_radar.header.frame_id = "mk_kalman";
			mk_radar.type = visualization_msgs::Marker::SPHERE_LIST;
			mk_radar.header.stamp = ros::Time::now();
			mk_radar.lifetime = ros::Duration();
			mk_radar.ns = "radar_filter";
			mk_radar.id = 0;

			mk_radar.scale.x = 2.1;
			mk_radar.scale.y = 2.1;
			mk_radar.scale.z = 2.1;

			mk_radar.color.r = 0.0f;
			mk_radar.color.g = 1.0f;
			mk_radar.color.b = 0.70f;
			mk_radar.color.a = 0.60;

	}
	
	void getXY(const platooning_perception::multican& msg)
	{
	
		radar_x.fill(0);
		radar_y.fill(0);

		int msg_size = 0;
		for(int i=0; i<42; i++){
			if(msg.can_msgs[i].id){msg_size++;}
		}

		for(int i=0;i<msg_size;i++){

			if((msg.can_msgs[i].id >= 257) && (msg.can_msgs[i].id <= 272))
			{
				int r_index = 0;
				int mov_flag = (msg.can_msgs[i].data[4] >> 6);

				r_index = msg.can_msgs[i].id - 257;
				radar_pos_x = (float)((((msg.can_msgs[i].data[2] & 0x7F)<<8) + msg.can_msgs[0].data[1])*0.01);
				radar_pos_y = (float)((((msg.can_msgs[i].data[4] & 0x3F)<<8) + msg.can_msgs[0].data[3])*(-0.01) + 81.91);

				if((radar_pos_x < 150) && (radar_pos_y > -10) && (radar_pos_y < 10)){
					if((mov_flag == 1) || (mov_flag == 3))
					{
						radar_x[r_index] = radar_pos_x;
						radar_y[r_index] = radar_pos_y;
						rx_float.push_back(radar_pos_x);
//						ry_float.push_back(radar_pos_y);

					}
				}
			}
		}
		for(int i=0; i<radar_y.size(); i++)
		{
			cout << radar_y[i] << ' ';
		}
		cout << endl;
		for(int i=0; i<ry_float.size(); i++)
		{
			cout << ry_float[i] << ' ';
		}
		cout << endl;
	}

	void Kalman()
	{
	
		int n = 3; // Number of states
		int m = 1; // Number of measurements
		double val =0;
		double dt = 1.0/30; // Time step

		Eigen::MatrixXd A(n, n); // System dynamics matrix
		Eigen::MatrixXd C(m, n); // Output matrix
		Eigen::MatrixXd Q(n, n); // Process noise covariance
		Eigen::MatrixXd R(m, m); // Measurement noise covariance
		Eigen::MatrixXd P(n, n); // Estimate error covariance

		// Discrete LTI projectile motion, measuring position only
//		A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
		A << 1, -0.2, 0,
		     0.2, 1, 0, 
			 0, 0, 1;
		C << 1, 0, 0;

		// Reasonable covariance matrices
		Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
		R << 5;
		P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

//		cout << "A: \n" << A << endl;
//		cout << "C: \n" << C << endl;
//		cout << "Q: \n" << Q << endl;
//		cout << "R: \n" << R << endl;
//		cout << "P: \n" << P << endl;

		// Construct the filter
		radar_kalman::KalmanFilter kf(dt, A, C, Q, R, P);

		// List of noisy position measurements (y)
//		vector<double> measurements = {
//		};

		// Best guess of initial states
		Eigen::VectorXd x0(n);
//		x0 << measurements[0], 0, -9.81;
		if(radar_y.size() > 0){
			x0 << radar_y[0], 0, 0;
			kf.init(dt, x0);

//			cout << '1' << endl;


			// Feed measurements into filter, output estimated states
			Eigen::VectorXd y(m);
//			cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << endl;
//			for(int i = 0; i < measurements.size(); i++) 
			for(int i = 0; i < radar_y.size(); i++) {
//				y << measurements[i];
//				y << ry_float[i];
//				cout << '2' << endl;
				if(radar_y[i] == 0) continue;
				else{
					y << radar_y[i];
					kf.update(y);
			//		cout << "y[" << i << "] = " << y.transpose()
			//			<< ", x_hat[" << i << "] = " << kf.state().transpose() << endl;
					cout <<'['<<i<<"] "<< kf.state()(0,0) << ' ';
					val = kf.state()(0,0);
					cout << val << ' ';
					ry_float.push_back(val);
				
				}
			}
			cout << "kalman end;" <<endl;
		}
	
	}
	
	void getMarker()
	{
		geometry_msgs::Point pt;

		for(int i=0; i<rx_float.size(); i++){
			pt.x = rx_float[i];
			pt.y = ry_float[i];
			mk_radar.points.push_back(pt);
			cout << ry_float[i]<< "=== " << endl;
		}
	//	cout << "radar------" << endl;
			
		ma.markers.push_back(mk_radar);

	}

	void Clear()
	{
		rx_float.clear();
		ry_float.clear();
		mk_radar.points.clear();
		ma.markers.clear();
	
	}

	void Callback(const platooning_perception::multican& msg)
	{
		initMarker();
		getXY(msg);
		Kalman();
		getMarker();

		pub.publish(ma);
		
		Clear();
		ROS_INFO("=====end=====");
	}

};
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar_kalman");
	ros::NodeHandle nh("~");
	radar_kalman::RadarKalman RK(nh);
	ros::spin();

	return 0;
}


/*
namespace adaptive_gating{

bool start_flag = false;

AdaptiveGating::AdaptiveGating(ros::NodeHandle node)
		: n(node)
{
	ROS_INFO("Start");
    sub = n.subscribe("/filtered_msgs",100, &AdaptiveGating::Callback, this);
    pub = n.advertise<visualization_msgs::MarkerArray>("/adaptive", 10);
}

AdaptiveGating::~AdaptiveGating()
{
}

void AdaptiveGating::init_LowPassFilter(){
	if(start_flag == false){
		for(int i=0; i < radar_y.size(); i++){
			filteredValue_x[i] = radar_x[i];
			filteredValue_y[i] = radar_y[i];
		}
		start_flag = true;
	}
}

void AdaptiveGating::LowPassFilter(){

	float w = 0.1;
	float sensorValue_x,sensorValue_y;

	for(int i=0; i < radar_y.size(); i++){
		if((filter_switch[i] == false)&&(radar_x[i] != 0)){
			filter_switch[i] = true;
			sensorValue_x = radar_x[i];
			sensorValue_y = radar_y[i];
			filteredValue_x[i] = sensorValue_x;
			filteredValue_y[i] = sensorValue_y;

		}
		else if(radar_x[i] == 0){
			filteredValue_x[i] = 0;
			filteredValue_y[i] = 0;
			filter_switch[i] = false;
		}
		
		if(filter_switch[i] == true){
			sensorValue_x = radar_x[i];
			sensorValue_y = radar_y[i];

			filteredValue_x[i] = filteredValue_x[i]*(1 - w) + sensorValue_x*w;
			filteredValue_y[i] = filteredValue_y[i]*(1 - w) + sensorValue_y*w;
			rx_float.push_back(filteredValue_x[i]);
			ry_float.push_back(filteredValue_y[i]);
		}

//		cout << "radar_x["<< i <<"] = " << radar_x[i] << " filtered_x = " << filteredValue_x[i] << endl;
//		cout << "radar_y["<< i <<"] = " << radar_y[i] << " filtered_y = " << filteredValue_y[i] << endl;
		
	}

}

//TEST FUNCTION//
void AdaptiveGating::print(){


		int n = 3; // Number of states
		int m = 1; // Number of measurements

		double dt = 1.0/30; // Time step

		Eigen::MatrixXd A(n, n); // System dynamics matrix
		Eigen::MatrixXd C(m, n); // Output matrix
		Eigen::MatrixXd Q(n, n); // Process noise covariance
		Eigen::MatrixXd R(m, m); // Measurement noise covariance
		Eigen::MatrixXd P(n, n); // Estimate error covariance

		// Discrete LTI projectile motion, measuring position only
		A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
		C << 1, 0, 0;

		// Reasonable covariance matrices
		Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
		R << 5;
		P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

//		cout << "A: \n" << A << endl;
//		cout << "C: \n" << C << endl;
//		cout << "Q: \n" << Q << endl;
//		cout << "R: \n" << R << endl;
//		cout << "P: \n" << P << endl;

		// Construct the filter
		adaptive_gating::KalmanFilter kf(dt, A, C, Q, R, P);

		// List of noisy position measurements (y)
//		vector<double> measurements = {
//		};

		// Best guess of initial states
		Eigen::VectorXd x0(n);
//		x0 << measurements[0], 0, -9.81;
		x0 << radar_y[0], 0, 0;
		kf.init(dt, x0);

		// Feed measurements into filter, output estimated states
		Eigen::VectorXd y(m);
//		cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << endl;
//		for(int i = 0; i < measurements.size(); i++) {
		for(int i = 0; i < radar_y.size(); i++) {
//				y << measurements[i];
				if(radar_y[i] == 0) continue;
				else{
					y << radar_y[i];
					kf.update(y);
//					cout << "y[" << i << "] = " << y.transpose()
//							<< ", x_hat[" << i << "] = " << kf.state().transpose() << endl;
				}
		}
}

void AdaptiveGating::getXY(const platooning_perception::multican& msg)
{
	radar_x.fill(0);
	radar_y.fill(0);
	camera_x.fill(0);
	camera_y.fill(0);
	camera_w.fill(0);
	fusion_x.fill(0);
	fusion_y.fill(0);

	int msg_size = 0;
	for(int i=0; i<42; i++){
		if(msg.can_msgs[i].id){msg_size++;}
	}

	for(int i=0;i<msg_size;i++){

		if((msg.can_msgs[i].id >= 257) && (msg.can_msgs[i].id <= 272))
		{
			int r_index = 0;
			int mov_flag = (msg.can_msgs[i].data[4] >> 6);
			
			r_index = msg.can_msgs[i].id - 257;
			radar_pos_x = (float)((((msg.can_msgs[i].data[2] & 0x7F)<<8) + msg.can_msgs[0].data[1])*0.01);
			radar_pos_y = (float)((((msg.can_msgs[i].data[4] & 0x3F)<<8) + msg.can_msgs[0].data[3])*(-0.01) + 81.91);

			if((radar_pos_x < 150) && (radar_pos_y > -10) && (radar_pos_y < 10)){
    			if((mov_flag == 1) || (mov_flag == 3))
    			{
    				radar_x[r_index] = radar_pos_x;
    				radar_y[r_index] = radar_pos_y;
    
    			}
			}
		}


		else if((msg.can_msgs[i].id >= 513) && (msg.can_msgs[i].id <= 528))
		{
			int f_index = 0;
			f_index = msg.can_msgs[i].id - 513;
			int f_sign = ((msg.can_msgs[i].data[3] & 0x10)>>4);

			fusion_pos_x = (float)(((msg.can_msgs[i].data[0])+((msg.can_msgs[i].data[1] & 0x7F)<<8))*0.01);

			if (f_sign == 0){
				fusion_pos_y = (float)(((msg.can_msgs[i].data[1]>>7) +(msg.can_msgs[i].data[2]<<1)+((msg.can_msgs[i].data[3] & 0x1F)<<9))*0.01);

			}
			else if(f_sign == 1){
				fusion_pos_y = (float)(16384 - ((msg.can_msgs[i].data[1]>>7) +(msg.can_msgs[i].data[2]<<1)+((msg.can_msgs[i].data[3] & 0x1F)<<9)))*(-0.01);
						}

			if((fusion_pos_x == 0) && (fusion_pos_y == 0)) continue;
			else{
				fusion_x[f_index] = fusion_pos_x;
				fusion_y[f_index] = fusion_pos_y;
				fx_float.push_back(fusion_pos_x);
				fy_float.push_back(fusion_pos_y);
			}
		}

		else if((msg.can_msgs[i].id >= 1841) && (msg.can_msgs[i].id <= 1850))
		{
			int c_index = 0;
			c_index = msg.can_msgs[i].id - 1841;
			int c_sign = (msg.can_msgs[i].data[3] & 0x20)>>5;
			float c_width = (float)((msg.can_msgs[i].data[6]>>2) + ((msg.can_msgs[i].data[7] & 0x01)<<6))*0.05;

			camera_pos_x = (float)(((msg.can_msgs[i].data[2] & 0x03)<<10) + (msg.can_msgs[i].data[1]<<2) + (msg.can_msgs[i].data[0]>>6))*0.05;

			if(c_sign == 0){
				camera_pos_y = (float)(((msg.can_msgs[i].data[3] & 0x3F)<<6) + (msg.can_msgs[i].data[2]>>2))*(-0.05);
			}
			else if(c_sign == 1){ 
				camera_pos_y = (float)(4096 - (((msg.can_msgs[i].data[3] & 0x3F)<<6) + (msg.can_msgs[i].data[2]>>2)))*(0.05);
			}
			
			if((camera_pos_x == 0) && (camera_pos_y == 0)) continue;
			else if((camera_pos_x < 150) && (camera_pos_y > -10) && (camera_pos_y < 10))
			{
				camera_x[c_index] = camera_pos_x;
				camera_y[c_index] = camera_pos_y;
				camera_w[c_index] = c_width;
				cx_float.push_back(camera_pos_x);
				cy_float.push_back(camera_pos_y);
			}
		}

	}
/////////printer//////////////////////////////////////////////////////
//		cout << "radar_size() : " << radar_x.size() << endl;
//
//		for(int i=0; i<16; i++){
//			printf("radar_pos_x[%d] : %f \n",i,radar_x[i]);
//			printf("radar_pos_y[%d] : %f \n",i,radar_y[i]);
//		}
//		printf("\n");

//		for(int i=0; i<10; i++){
//			printf("camera_pos_x[%d] : %f \n",i,camera_x[i]);
//			printf("camera_pos_y[%d] : %f \n",i,camera_y[i]);
//			printf("camera_pos_w[%d] : %f \n",i,camera_w[i]);
//		}
//		printf("\n");

//		for(int i=0; i<16; i++){
//			printf("fusion_pos_x[%d] : %f \n",i,fusion_x[i]);
//			printf("fusion_pos_y[%d] : %f \n",i,fusion_y[i]);
//		}

//		printf("radar_pos_x[%d] \n",sizeof(radar_x)/sizeof(radar_x[0]));
//		printf("radar_pos_y[%d] \n",sizeof(radar_y)/sizeof(radar_x[0]));
//		printf("camera_pos_x[%d] \n",sizeof(camera_x)/sizeof(radar_x[0]));
//		printf("camera_pos_y[%d] \n",sizeof(camera_y)/sizeof(radar_x[0]));
//		printf("fusion_pos_x[%d] \n",sizeof(fusion_x)/sizeof(radar_x[0]));
//		printf("fusion_pos_y[%d] \n",sizeof(fusion_y)/sizeof(radar_x[0]));

//		printf("=============end==============\n");

////////////////////////////////////////////////////////////////

}
	

void AdaptiveGating::getGateSize()
{
// th_x = 60 / th_y = 80
	for(int i=0; i < radar_x.size(); i++)
	{
		if((radar_x[i] > 0) && (radar_x[i] <= threshold_x)){
			radar_gate_xx.push_back((0.0333*radar_x[i]) + 3);
		}

		else if(radar_x[i] > threshold_x){
			radar_gate_xx.push_back(5);
		}
		if((radar_x[i] > 0) && (radar_x[i] < threshold_x)){
			radar_gate_yy.push_back((radar_x[i]*0.05) + 7);
		}
		else if(radar_x[i] > threshold_x){
			radar_gate_yy.push_back(10);
		}
//		printf("radar_gate_x[%d] : %f \n",i,radar_gate_x[i]);
//		printf("radar_gate_y[%d] : %f \n",i,radar_gate_y[i]);
	}


	for(int i=0; i < camera_x.size(); i++)
	{
		if((camera_x[i] > 0) && (camera_x[i] <= threshold_x)){
			camera_gate_xx.push_back((0.117*camera_x[i]) + 5);
		}
		else if(camera_x[i] > threshold_x){
			camera_gate_xx.push_back(12);
		}
		if(camera_x[i] > 0){
			camera_gate_yy.push_back(camera_w[i]);
		}
//			printf("camera_gate_x[%d] : %f \n",i,camera_gate_x[i]);
//			printf("camera_gate_y[%d] : %f \n",i,camera_gate_y[i]);
	}

//		printf("========================================\n");


}


void AdaptiveGating::getLine()
{
	geometry_msgs::Point pt;

	// radar_line & origin
	pt.x = 4.66;
	pt.y = 10;
	mk_line.points.push_back(pt);
	pt.x = 0;
	pt.y = 0;
	mk_line.points.push_back(pt);
	mk_line.points.push_back(pt);
	mk_origin.points.push_back(pt);
	pt.x = 4.66;
	pt.y = -10;
	mk_line.points.push_back(pt);

	// road_line
	pt.x = 100;
	pt.y = 2;
	mk_line.points.push_back(pt);
	pt.x = 0.9326;
	pt.y = 2;
	mk_line.points.push_back(pt);
	pt.x = 100;
	pt.y = -2;
	mk_line.points.push_back(pt);
	pt.x = 0.9326;
	pt.y = -2;
	mk_line.points.push_back(pt);

	for(int i=0; i<4; i++){
		for(int j=0; j<20; j++){
			pt.x = 1 + 5*j;
			pt.y = -6 + 4*i;
			mk_line2.points.push_back(pt);
		}
	}

	ma.markers.push_back(mk_origin);
	ma.markers.push_back(mk_line);
	ma.markers.push_back(mk_line2);
}
void AdaptiveGating::getMarker()
{
	geometry_msgs::Point pt;


	for(int i=0; i<cx_float.size(); i++){
		pt.x = cx_float[i];
		pt.y = cy_float[i];
		mk_camera.points.push_back(pt);
//		cout << cx_float[i] << endl;
	}	
//	cout << "camera------" << endl;


	for(int i=0; i<rx_float.size(); i++){
		if(rx_float[i] > 1){
			pt.x = rx_float[i];
			pt.y = ry_float[i];
			mk_radar.points.push_back(pt);
		}
//		cout << rx_float[i] << endl;
	}
//	cout << "radar------" << endl;

	for(int i=0; i<fx_float.size(); i++){
		pt.x = fx_float[i];
		pt.y = fy_float[i];
		mk_fusion.points.push_back(pt);
//		cout << fx_float[i] << endl;
	}
//	cout << "fusion------" << endl;
	
	for(int i=0; i<ffx_float.size(); i++){
		pt.x = ffx_float[i];
		pt.y = ffy_float[i];
		mk_ffusion.points.push_back(pt);
//		cout <<"ffx:"<< ffx_float[i] << " ffy:" << ffy_float[i] << " i=" << i << endl;
	}
//	cout << "ffusion------" << endl;

	ma.markers.push_back(mk_camera);
	ma.markers.push_back(mk_radar);
//	ma.markers.push_back(mk_fusion);
	ma.markers.push_back(mk_ffusion);

}

void AdaptiveGating::initMarker()
{
//-------------------------------------------------origin
	mk_origin.header.frame_id = "mk";
	mk_origin.type = visualization_msgs::Marker::CUBE_LIST;
	mk_origin.header.stamp = ros::Time::now();
	mk_origin.lifetime = ros::Duration();
	mk_origin.ns = "mk_origin";
	mk_origin.id = 0;

	mk_origin.scale.x = 2.1;
	mk_origin.scale.y = 2.1;
	mk_origin.scale.z = 2.1;

	mk_origin.color.r = 1.0f;
	mk_origin.color.g = 1.0f;
	mk_origin.color.b = 1.0f;
	mk_origin.color.a = 1.0;

//-------------------------------------------------line2
	mk_line2.header.frame_id = "mk";
	mk_line2.type = visualization_msgs::Marker::CUBE_LIST;
	mk_line2.header.stamp = ros::Time::now();
	mk_line2.lifetime = ros::Duration();
	mk_line2.ns = "mk_line2";
	mk_line2.id = 6;

	mk_line2.scale.x = 3.4;
	mk_line2.scale.y = 0.50;
	mk_line2.scale.z = 0.01;

	mk_line2.color.r = 1.0f;
	mk_line2.color.g = 1.0f;
	mk_line2.color.b = 1.0f;
	mk_line2.color.a = 1.0;

//-------------------------------------------------radar_line
	mk_line.header.frame_id = "mk";
	mk_line.type = visualization_msgs::Marker::LINE_LIST;
	mk_line.header.stamp = ros::Time::now();
	mk_line.lifetime = ros::Duration();
	mk_line.ns = "mk_line";
	mk_line.id = 5;

	mk_line.scale.x = 0.1;

	mk_line.color.r = 1.0f;
//	mk_line.color.g = 1.0f;
//	mk_line.color.b = 1.0f;
	mk_line.color.a = 0.60;

//-------------------------------------------------camera
	mk_camera.header.frame_id = "mk";
	mk_camera.type = visualization_msgs::Marker::SPHERE_LIST;
    mk_camera.header.stamp = ros::Time::now();
	mk_camera.lifetime = ros::Duration();
	mk_camera.ns = "mk_camera";
	mk_camera.id = 1;

	mk_camera.scale.x = 2.1;
	mk_camera.scale.y = 2.1;
	mk_camera.scale.z = 2.1;

	mk_camera.color.r = 0.0f;
	mk_camera.color.g = 1.0f;
	mk_camera.color.b = 0.0f;
	mk_camera.color.a = 0.60;

//-------------------------------------------------radar
	mk_radar.header.frame_id = "mk";
	mk_radar.type = visualization_msgs::Marker::SPHERE_LIST;
    mk_radar.header.stamp = ros::Time::now();
	mk_radar.lifetime = ros::Duration();
	mk_radar.ns = "mk_radar";
	mk_radar.id = 2;

	mk_radar.scale.x = 2.1;
	mk_radar.scale.y = 2.1;
	mk_radar.scale.z = 2.1;

	mk_radar.color.r = 1.0f;
	mk_radar.color.g = 0.0f;
	mk_radar.color.b = 0.0f;
	mk_radar.color.a = 0.60;

//-------------------------------------------------fusion
	mk_fusion.header.frame_id = "mk";
	mk_fusion.type = visualization_msgs::Marker::CUBE_LIST;
	mk_fusion.header.stamp = ros::Time::now();
	mk_fusion.lifetime = ros::Duration();
	mk_fusion.ns = "mk_fusion";
	mk_fusion.id = 3;

	mk_fusion.scale.x = 5.1;
	mk_fusion.scale.y = 2.1;
	mk_fusion.scale.z = 2.1;

	mk_fusion.color.r = 0.60f;
	mk_fusion.color.g = 0.60f;
	mk_fusion.color.b = 1.0f;
	mk_fusion.color.a = 0.6;

//-------------------------------------------------Our fusion
	mk_ffusion.header.frame_id = "mk";
	mk_ffusion.type = visualization_msgs::Marker::CUBE_LIST;
	mk_ffusion.header.stamp = ros::Time::now();
	mk_ffusion.lifetime = ros::Duration();
	mk_ffusion.ns = "mk_ffusion";
	mk_ffusion.id = 4;

	mk_ffusion.scale.x = 5.1;
	mk_ffusion.scale.y = 2.1;
	mk_ffusion.scale.z = 2.1;

	mk_ffusion.color.r = 1.0f;
	mk_ffusion.color.g = 1.0f;
	mk_ffusion.color.b = 0.0f;
	mk_ffusion.color.a = 0.6;

}

void AdaptiveGating::clearVec()
{

	cx_float.clear();
	cy_float.clear();
	rx_float.clear();
	ry_float.clear();
	fx_float.clear();
	fy_float.clear();
	ffx_float.clear();
	ffy_float.clear();

	camera_gate_xx.clear();
	camera_gate_yy.clear();
	radar_gate_xx.clear();
	radar_gate_yy.clear();

	id_both_n.clear();
	id_both_m.clear();
	id_camera_only.clear();
	id_radar_only.clear();

	mk_origin.points.clear();
	mk_line.points.clear();
	mk_line2.points.clear();
	mk_camera.points.clear();
	mk_radar.points.clear();
	mk_fusion.points.clear();
	mk_ffusion.points.clear();
	ma.markers.clear();
}

void AdaptiveGating::Fusion()
{
	int lr = rx_float.size();
	int lc = cx_float.size();
	int l_max, l_min, n, m;
	bool iou_check = false;
	float iou_x, iou_y;

	vector<float> dist;

	float cx_max, cy_max;
	float cx_min, cy_min;
	float rx_max, ry_max;
	float rx_min, ry_min;

	float x_max, y_max;
	float x_min, y_min;

	lr > lc ? l_max = lr : l_max = lc;	
	lr < lc ? l_min = lr : l_min = lc;	

	cout <<"lr:" << lr << " lc:" << lc << " l_max:" << l_max << " l_min:" << l_min << endl;
	for(int i=0; i < lr; i++){
		cout << rx_float[i] << ',';
		cout << ry_float[i] << " / ";

	}
	cout << endl;
	for(int i=0; i < lc; i++){
		cout << cx_float[i] << ',';
		cout << cy_float[i] << " / ";
	}
	cout << endl;

	if((lc != 0) && (lr != 0)){
		for(int i=0; i < l_max; i++){
			for(int j=0; j < l_min; j++){
				if(lc >= lr){ n=i; m=j;}
				else if(lc < lr){ m=i; n=j;}

//				cout << "cam_x :" << cx_float[n] << " cam_y :" << cy_float[n] << endl;
//				cout << "radar_x :" << rx_float[m] << " radar_y :" << ry_float[m] << endl;
//				cout << "n: "<<n << " m: " << m << endl;	
				iou_check = false;
				
				cx_max = cx_float[n] + camera_gate_xx[n]/2;
				cx_min = cx_float[n] - camera_gate_xx[n]/2;
				cy_max = cy_float[n] + camera_gate_yy[n]/2;
				cy_min = cy_float[n] - camera_gate_yy[n]/2;

				rx_max = rx_float[m] + radar_gate_xx[m]/2;
				rx_min = rx_float[m] - radar_gate_xx[m]/2;
				ry_max = ry_float[m] + radar_gate_yy[m]/2;
				ry_min = ry_float[m] - radar_gate_yy[m]/2;
				

				x_max = cx_max > rx_max ? rx_max : cx_max;
				x_min = cx_min > rx_min ? cx_min : rx_min;
				y_max = cy_max > ry_max ? ry_max : cy_max;
				y_min = cy_min > ry_min ? cy_min : ry_min;
//				cout << "x_max:"  << x_max << " x_min:" << x_min << " y_max:" << y_max << " y_min:" << y_min << endl;

				iou_x = x_max - x_min;
				iou_y = y_max - y_min;
//				cout << "iou_x:"<<iou_x << " iou_y:" << iou_y << endl;


				if((iou_x > 0) && (iou_y > 0)){
					iou_check = true;
				}
//				cout << "iou_check "<< iou_check << endl;

				if(iou_check == true) // meas = 3_both_fusion
				{
//					cout << "<<both fusion!>>\n" << endl;
					id_both_n.push_back(n);
					id_both_m.push_back(m);
				}

				else if((iou_check == false) && (n > m)) // meas = 2_camera_only
				{
//					cout << "<<camera only fusion!>>\n" << endl;
					id_camera_only.push_back(n);
				}

				else if((iou_check == false) && (m >= n)) // meas = 1_radar_only
				{
					if(m == n){
						id_camera_only.push_back(n);
						id_radar_only.push_back(m);				
//						cout << "<<each fusion!>>\n" << endl;
					}
					else{
//						cout << "<<radar only fusion!>>\n" << endl;
						id_radar_only.push_back(m);
					}
				}

			}
//			cout << "=========" << endl;
		}
	}

	else if((lc == 0) && (lr != 0)){
		for(int i = 0; i < rx_float.size(); i++){
			ffx_float.push_back(rx_float[i]);
			ffy_float.push_back(ry_float[i]);
		}
	}

	else if((lr == 0) && (lc != 0)){
		for(int i = 0; i < cx_float.size(); i++){
			ffx_float.push_back(cx_float[i]);
			ffy_float.push_back(cy_float[i]);
		}
	}

	sort(id_camera_only.begin(),id_camera_only.end());
	sort(id_radar_only.begin(),id_radar_only.end());

	id_camera_only.erase(unique(id_camera_only.begin(),id_camera_only.end()),id_camera_only.end());
	id_radar_only.erase(unique(id_radar_only.begin(),id_radar_only.end()),id_radar_only.end());
	
	int min_n, min_m;
	float min_dist;
	cout << id_both_n.size() << ' ' << id_both_m.size() << endl;
	for(int i = 0; i < id_both_n.size(); i++){
		for(int j = 0; j < id_both_m.size(); j++){

			dist.push_back( sqrt(pow(cx_float[id_both_n[i]]-rx_float[id_both_m[j]],2)+pow(cy_float[id_both_n[i]]-ry_float[id_both_m[j]],2)) );
			cout << "distance["<<j << "] : " <<dist[j] << endl;
		}
		min_dist = dist[0];
		min_n = id_both_n[i];
		min_m = id_both_m[0];

		for(int j = 0; j < id_both_m.size(); j++){
			if(dist[j] < min_dist){
				min_dist = dist[j];
				min_n = id_both_n[i];
				min_m = id_both_m[j];
			}
		}

		ffx_float.push_back(cx_float[min_n]*0.365 + rx_float[min_m]*0.635);
		ffy_float.push_back(cy_float[min_n]*0.980 + ry_float[min_m]*0.020);

//		cout << ffx_float[i] << ' ' << ffy_float[i] << endl;
		dist.clear();	
	}
	
	auto endx = ffx_float.end();
	auto endy = ffy_float.end();
	for(auto it = ffx_float.begin(); it != endx; ++it){
		endx = remove(it+1, endx, *it);
	}
	for(auto it = ffy_float.begin(); it != endy; ++it){
		endy = remove(it+1, endy, *it);
	}
	ffx_float.erase(endx,ffx_float.end());	
	ffy_float.erase(endy,ffy_float.end());	

	for(int i = 0; i<ffx_float.size(); i++){
		cout << ffx_float[i] << ' ' << ffy_float[i] << endl;
	}

	for(int i = 0; i < id_camera_only.size(); i++){
		bool cam_only = true;

		for(int j = 0; j < id_both_n.size(); j++){
			if(id_camera_only[i] == id_both_n[j]){
				cam_only = false;
				break;}
		}	
		if(cam_only == true){
			ffx_float.push_back(cx_float[id_camera_only[i]]);
			ffy_float.push_back(cy_float[id_camera_only[i]]);
//			cout << "camera_id = " << id_camera_only[i] << endl;
		}
	}
	for(int i = 0; i < id_radar_only.size(); i++){
		bool radar_only = true;
		
		for(int j = 0; j < id_both_m.size(); j++){
			if(id_radar_only[i] == id_both_m[j]){
				radar_only = false;
				break;}
		}
		if(radar_only == true){
			ffx_float.push_back(rx_float[id_radar_only[i]]);
			ffy_float.push_back(ry_float[id_radar_only[i]]);
//			cout << "radar_id = " << id_radar_only[i] << endl;
		}
	}

	cout << "radar info" << endl;
	for(int i=0; i<rx_float.size(); i++){
		cout << "rx : "<<rx_float[i] << " / ry : " << ry_float[i] << endl;
	
	}

	cout << "======= FUSION END ======" << endl;
}


void AdaptiveGating::Callback(const platooning_perception::multican& msg)
{

	initMarker();	
	
	getXY(msg);
	init_LowPassFilter();
	LowPassFilter();
	getGateSize();
	
	Fusion();

	getLine();
	getMarker();

	pub.publish(ma);
	
	clearVec();

}

}
*/
