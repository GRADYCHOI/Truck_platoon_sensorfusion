#include <ros/ros.h>
#include <iostream>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <cmath>
#include <time.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <platooning_perception/can.h>
#include <platooning_perception/multican.h>

#include "platooning_perception/kalman.hpp"
#include "platooning_perception/AdaptiveGating.hpp"


double gettimeafterboot()
{
	struct timespec time_after_boot;
	clock_gettime(CLOCK_MONOTONIC,&time_after_boot);
	return (time_after_boot.tv_sec*1000+time_after_boot.tv_nsec*0.000001);
}


using namespace std;

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

	float w = 0.04;
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


void AdaptiveGating::getXY(const platooning_perception::multican& msg)
{
//	double current_time = gettimeafterboot();

//	ROS_INFO("-----------------start");

	radar_x.fill(0);
	radar_y.fill(0);
	camera_x.fill(0);
	camera_y.fill(0);
	camera_w.fill(0);
	fusion_x.fill(0);
	fusion_y.fill(0);

	int msg_size = 0;
	for(int i=0; i<50; i++){ // Warning!! See msg size
//		cout << msg.can_msgs[i].id << ' ';
		if((msg.can_msgs[i].id >= 257) && (msg.can_msgs[i].id <= 272)){msg_size++;}
		else if((msg.can_msgs[i].id >= 513) && (msg.can_msgs[i].id <= 528)){msg_size++;}
		else if((msg.can_msgs[i].id >= 1841) && (msg.can_msgs[i].id <= 1850)){msg_size++;}
		else if((msg.can_msgs[i].id >= 0x766) && (msg.can_msgs[i].id <= 0x769 )){msg_size++;}
	}
//	cout <<endl<<"msg_size : "<< msg_size << endl;
//	cout << "sizeof msg : " << sizeof(msg) << endl;
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

		//Lane
		else if((msg.can_msgs[i].id >= 0x766) && (msg.can_msgs[i].id <= 0x769 ))
		{

			int d_sign = (msg.can_msgs[i].data[6] & 0x08)>>3;
			int c_sign = (msg.can_msgs[i].data[4] & 0x08)>>3;
			int p_sign = (msg.can_msgs[i].data[2] & 0x08)>>3;

			if(msg.can_msgs[i].id == 0x766){ 
    			if(d_sign == 0){
    				L_lane_curvature_derivative = (float)(((msg.can_msgs[i].data[4] & 0xF0)>>4) + (msg.can_msgs[i].data[5]<<4) + ((msg.can_msgs[i].data[6] & 0x0F)<<12)) * 4 / pow(10,9);}
    			else if(d_sign == 1){
    				L_lane_curvature_derivative = (float)(65536 - (((msg.can_msgs[i].data[4] & 0xF0)>>4) + (msg.can_msgs[i].data[5]<<4) + ((msg.can_msgs[i].data[6] & 0x0F)<<12))) * (-4) / pow(10,9);}
//    			cout << "d_sign :"<<d_sign<<" L_lane_curve_de : "<< L_lane_curvature_derivative << endl;
    
    			if(c_sign == 0){
    				L_lane_curvature = (float)(((msg.can_msgs[i].data[2] & 0xF0)>>4) + (msg.can_msgs[i].data[3]<<4) + ((msg.can_msgs[i].data[4] & 0x0F)<<12)) / pow(10,6);}
    			else if(c_sign == 1){
    				L_lane_curvature = (float)(65536 - (((msg.can_msgs[i].data[2] & 0xF0)>>4) + (msg.can_msgs[i].data[3]<<4) + ((msg.can_msgs[i].data[4] & 0x0F)<<12))) * (-1) /pow(10,6);}
//    			cout << "c_sign :"<<c_sign<<" L_lane_curve : " << L_lane_curvature << endl;
    			
    			if(p_sign == 0){
    				L_lane_position = (float)((msg.can_msgs[i].data[1]) + ((msg.can_msgs[i].data[2] & 0x0F)<<8)) *0.01;}
    			else if(p_sign == 1){
    				L_lane_position = (float)(4096 - ((msg.can_msgs[i].data[1]) + ((msg.can_msgs[i].data[2] & 0x0F)<<8))) *(-0.01);}
//    			cout << "p_sign :"<<p_sign<<" L_lane_pos :" << L_lane_position << endl;
			}

			else if(msg.can_msgs[i].id == 0x768){ 
    			if(d_sign == 0){
    				R_lane_curvature_derivative = (float)(((msg.can_msgs[i].data[4] & 0xF0)>>4) + (msg.can_msgs[i].data[5]<<4) + ((msg.can_msgs[i].data[6] & 0x0F)<<12)) * 4 / pow(10,9);}
    			else if(d_sign == 1){
    				R_lane_curvature_derivative = (float)(65536 - (((msg.can_msgs[i].data[4] & 0xF0)>>4) + (msg.can_msgs[i].data[5]<<4) + ((msg.can_msgs[i].data[6] & 0x0F)<<12))) * (-4) / pow(10,9);}
//    			cout << "d_sign :"<<d_sign<<" R_lane_curve_de : "<< R_lane_curvature_derivative << endl;
    
    			if(c_sign == 0){
    				R_lane_curvature = (float)(((msg.can_msgs[i].data[2] & 0xF0)>>4) + (msg.can_msgs[i].data[3]<<4) + ((msg.can_msgs[i].data[4] & 0x0F)<<12)) / pow(10,6);}
    			else if(c_sign == 1){
    				R_lane_curvature = (float)(65536 - (((msg.can_msgs[i].data[2] & 0xF0)>>4) + (msg.can_msgs[i].data[3]<<4) + ((msg.can_msgs[i].data[4] & 0x0F)<<12))) * (-1) /pow(10,6);}
//    			cout << "c_sign :"<<c_sign<<" R_lane_curve : " << R_lane_curvature << endl;
    			
    			if(p_sign == 0){
    				R_lane_position = (float)((msg.can_msgs[i].data[1]) + ((msg.can_msgs[i].data[2] & 0x0F)<<8)) *0.01;}
    			else if(p_sign == 1){
    				R_lane_position = (float)(4096 - ((msg.can_msgs[i].data[1]) + ((msg.can_msgs[i].data[2] & 0x0F)<<8))) *(-0.01);}
//    			cout << "p_sign :"<<p_sign<<" R_lane_pos :" << R_lane_position << endl;
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

//	double last_time = gettimeafterboot();
//	double dt = last_time - current_time;
//	ROS_INFO("-----------------end");
//	printf("dt = %lf \n", dt);

}
	
//	double current_time = gettimeafterboot();
//	double last_time = gettimeafterboot();
//	double dt = last_time - current_time;
//	printf("dt = %lf \n", dt);

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

	// origin(EGO)
	pt.x = 0;
	pt.y = 0;
	mk_origin.points.push_back(pt);

	//Left_Lane
	for(int i =0; i<100; i++){
		for(int j= 0; j<10; j++){
			float y;
			int x = i + j*0.1;
			y = L_lane_curvature_derivative*x*x*x + L_lane_curvature*x*x + L_lane_position;
	
			pt.x = x;
			pt.y = -y;
			mk_L_lane.points.push_back(pt);
		}
	}

	//Right_Lane
	for(int i =0; i<100; i++){
		for(int j= 0; j<10; j++){
			float y;
			int x = i + j*0.1;
			y = R_lane_curvature_derivative*x*x*x + R_lane_curvature*x*x + R_lane_position;
	
			pt.x = x;
			pt.y = -y;
			mk_R_lane.points.push_back(pt);
		}
	}

//	//Next L_Lane
//	for(int i =0; i<100; i++){
//		for(int j= 0; j<10; j++){
//			float y;
//			int x = i + j*0.1;
//			y = L_lane_curvature_derivative*x*x*x + L_lane_curvature*x*x + L_lane_position - (fabs(L_lane_position)+fabs(R_lane_position));
//	
//			pt.x = x;
//			pt.y = -y;
//			mk_LL_lane.points.push_back(pt);
//		}
//	}
//
//	//Next R_Lane
//	for(int i =0; i<100; i++){
//		for(int j= 0; j<10; j++){
//			float y;
//			int x = i + j*0.1;
//			y = R_lane_curvature_derivative*x*x*x + R_lane_curvature*x*x + R_lane_position + (fabs(L_lane_position)+fabs(R_lane_position));
//	
//			pt.x = x;
//			pt.y = -y;
//			mk_RR_lane.points.push_back(pt);
//		}
//	}


//	for(int i=0; i<4; i++){
//		for(int j=0; j<20; j++){
//			pt.x = 1 + 5*j;
//			pt.y = -6 + 4*i;
//			mk_lane2.points.push_back(pt);
//		}
//	}

	ma.markers.push_back(mk_origin);
	ma.markers.push_back(mk_L_lane);
	ma.markers.push_back(mk_R_lane);
//	ma.markers.push_back(mk_LL_lane);
//	ma.markers.push_back(mk_RR_lane);
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
		pt.x = rx_float[i];
		pt.y = ry_float[i];
		mk_radar.points.push_back(pt);
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
	ma.markers.push_back(mk_fusion);
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

//-------------------------------------------------L_lane
	mk_L_lane.header.frame_id = "mk";
	mk_L_lane.type = visualization_msgs::Marker::LINE_STRIP;
	mk_L_lane.header.stamp = ros::Time::now();
	mk_L_lane.lifetime = ros::Duration();
	mk_L_lane.ns = "mk_L_lane";
	mk_L_lane.id = 5;

	mk_L_lane.scale.x = 0.5;

	mk_L_lane.color.r = 1.0f;
	mk_L_lane.color.g = 1.0f;
	mk_L_lane.color.b = 1.0f;
	mk_L_lane.color.a = 0.60;

//-------------------------------------------------R_lane
	mk_R_lane.header.frame_id = "mk";
	mk_R_lane.type = visualization_msgs::Marker::LINE_STRIP;
	mk_R_lane.header.stamp = ros::Time::now();
	mk_R_lane.lifetime = ros::Duration();
	mk_R_lane.ns = "mk_R_lane";
	mk_R_lane.id = 6;

	mk_R_lane.scale.x = 0.5;

	mk_R_lane.color.r = 1.0f;
	mk_R_lane.color.g = 1.0f;
	mk_R_lane.color.b = 1.0f;
	mk_R_lane.color.a = 0.60;

////-------------------------------------------------LL_lane
//	mk_LL_lane.header.frame_id = "mk";
//	mk_LL_lane.type = visualization_msgs::Marker::LINE_STRIP;
//	mk_LL_lane.header.stamp = ros::Time::now();
//	mk_LL_lane.lifetime = ros::Duration();
//	mk_LL_lane.ns = "mk_LL_lane";
//	mk_LL_lane.id = 7;
//
//	mk_LL_lane.scale.x = 0.5;
//
//	mk_LL_lane.color.r = 1.0f;
//	mk_LL_lane.color.g = 1.0f;
//	mk_LL_lane.color.b = 1.0f;
//	mk_LL_lane.color.a = 0.60;
//
////-------------------------------------------------RR_lane
//	mk_RR_lane.header.frame_id = "mk";
//	mk_RR_lane.type = visualization_msgs::Marker::LINE_STRIP;
//	mk_RR_lane.header.stamp = ros::Time::now();
//	mk_RR_lane.lifetime = ros::Duration();
//	mk_RR_lane.ns = "mk_RR_lane";
//	mk_RR_lane.id = 8;
//
//	mk_RR_lane.scale.x = 0.5;
//
//	mk_RR_lane.color.r = 1.0f;
//	mk_RR_lane.color.g = 1.0f;
//	mk_RR_lane.color.b = 1.0f;
//	mk_RR_lane.color.a = 0.60;

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
	mk_L_lane.points.clear();
//	mk_LL_lane.points.clear();
	mk_R_lane.points.clear();
//	mk_RR_lane.points.clear();
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
/*
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
*/
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
//			cout << "distance["<<j << "] : " <<dist[j] << endl;
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

//	for(int i = 0; i<ffx_float.size(); i++){
//		cout << ffx_float[i] << ' ' << ffy_float[i] << endl;
//	}

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
/*
	cout << "radar info" << endl;
	for(int i=0; i<rx_float.size(); i++){
		cout << "rx : "<<rx_float[i] << " / ry : " << ry_float[i] << endl;
	
	}
*/
	cout << "======= FUSION END ======" << endl;
}

void AdaptiveGating::getTarget()
{
	float L_lane, R_lane;
	float x;
	float temp;
	vector<float> inlane_x;

	for(int i=0; i<ffx_float.size(); i++)
	{
		x = ffx_float[i];
		L_lane = L_lane_curvature_derivative*x*x*x + L_lane_curvature*x*x + L_lane_position;
		R_lane = R_lane_curvature_derivative*x*x*x + R_lane_curvature*x*x + R_lane_position;
		if((ffy_float[i] > L_lane) && (ffy_float[i] < R_lane))
		{
			inlane_x.push_back(x);
			cout << "in Lane : "<< x << endl;
		}
	}

}

void AdaptiveGating::Callback(const platooning_perception::multican& msg)
{

	double current_time = gettimeafterboot();

	initMarker();	
	
	getXY(msg);
	init_LowPassFilter();
	LowPassFilter();
	getGateSize();
	
	Fusion();

	getLine();
	getMarker();

	getTarget();
	pub.publish(ma);
	
	clearVec();


	double last_time = gettimeafterboot();
//	double dt = last_time - current_time;
//	printf("dt = %lf \n", dt);
	printf("----------------LOOP END-----------------\n");


}

}
