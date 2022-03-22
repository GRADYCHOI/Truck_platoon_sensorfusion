#define _GNU_SOURCE

#include <cstdlib>
#include <cstring>
#include <iostream>

extern "C"
{
#include "lib.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
}

//ros
#include <ros/ros.h>
#include <platooning_perception/can.h>
#include <platooning_perception/multican.h>
#include <can_msgs/Frame.h>

using namespace std;

array<float, 16> check_radar_id = {0};
int radar_index, lane_index, camera_index, fusion_index, esr_index;
//int check_radar_id[16]={0,};
int check_lane_id[4]={0,};
int check_camera_id[10] = {0,};
int check_fusion_id[16] = {0,};
int check_esr_id[64] = {0,};

int radar_count = 0 , camera_count = 0, fusion_count = 0, lane_count = 0, esr_count =0;
int pub_count = 0, pub_couont1 = 0, pub_count2 = 0, pub_count3 = 0;
bool isRadarFull = false, isCameraFull = false, isFusionFull = false, isLaneFull = false, isESRFull = false;
int count = 0;
double last_time = 0;
can_msgs::Frame msg;
platooning_perception::can can_data;
platooning_perception::can can_data2;
platooning_perception::multican multican_data;
platooning_perception::multican multican_data2;

double gettimeafterboot()
{
	struct timespec time_after_boot;
	clock_gettime(CLOCK_MONOTONIC,&time_after_boot);
	return (time_after_boot.tv_sec*1000+time_after_boot.tv_nsec*0.000001);
}

bool check_id(int id)
{
	bool check = false;

	if((id >= 257) && (id <= 272)){check = true;} // LGE RADAR
	else if((id >= 513) && (id <= 528)){check = true;} // LGE FUSION
	else if((id >= 1841) && (id <= 1850)){check = true;} // MOBILEYE
	else if((id >= 0x766) && (id <= 0x769 )){check = true;} // MOBILEYE_LANE
//	else if((id >= 1280) && (id <= 1343 )){check = true;}

	return check;
}

void callback(const can_msgs::Frame msg){
	
//	printf("============receive start============\n");
//	ROS_INFO("======start======");
	can_data.id = msg.id;
	can_data.dlc = msg.dlc;
	can_data.data = msg.data;

//	printf("current ID : %d\n", can_data.id);

	if((can_data.id >= 257) && (can_data.id <= 272) && (isRadarFull == false))
	{
		
		if(can_data.id == 257){
			radar_count = 0;
//			memset(check_radar_id, 0, sizeof(check_radar_id));
			check_radar_id.fill(0);
		}
            
        radar_index = can_data.id-257;
		check_radar_id[radar_index] = 1;


//    	printf("Radar ID : %02X %d\n", can_data.id,radar_count);
//    	printf("radar_count : %d\n",radar_count);
	    multican_data.can_msgs.push_back(can_data);

   		if(check_radar_id[radar_index] == 1) radar_count++;

//		for(int i=0; i<16; i++){
//			printf("index%d %d \n",i,check_radar_id[i]);
//		}
    	if(radar_count == 16)
    	{
    		isRadarFull = true;
    	}
        
    }

	else if((can_data.id >= 513) && (can_data.id <= 528) && (isFusionFull == false))
	{
		if(can_data.id == 513) fusion_count = 0;
		
		fusion_index = can_data.id-513;
		check_fusion_id[fusion_index] = 1;
		
//		printf("Fusion ID : %02X\n", can_data.id);
//		printf("fusion_count : %d\n",fusion_count);
		
		multican_data.can_msgs.push_back(can_data);
		
		if(check_fusion_id[fusion_index] == 1) fusion_count++;

		if(fusion_count == 16)
		{
			isFusionFull =true;
			fusion_count = 0;
		}
	
	}

	else if((can_data.id >= 1841) && (can_data.id <= 1850) && (isCameraFull == false))
	{
		cout << "can id = "<< can_data.id << endl;
		if(can_data.id == 1841) camera_count = 0;
		
		camera_index = can_data.id-1841;
		check_camera_id[camera_index] = 1;
		
//		printf("Camera ID : %02X %d\n", can_data.id,camera_count);
//		printf("camera_count : %d\n",camera_count);
		
		multican_data.can_msgs.push_back(can_data);
		
		if(check_camera_id[camera_index] == 1) camera_count++;

		if(camera_count == 10)
		{
			isCameraFull =true;
			camera_count = 0;
		}
	
	}

	else if((can_data.id >= 1894) && (can_data.id <= 1897) && (isLaneFull == false))
	{
		if(can_data.id == 1894) lane_count = 0;
		
		lane_index = can_data.id-1894;
		check_lane_id[lane_index] = 1;
		
//		printf("lane ID : %02X %d\n", can_data.id, lane_count);
//		printf("lane_count : %d\n",lane_count);
		
		multican_data.can_msgs.push_back(can_data);
		
		if(check_lane_id[lane_index] == 1) lane_count++;

		if(lane_count == 4)
		{
			isLaneFull =true;
			lane_count = 0;
		}
		
//		printf(isLaneFull ? "true" : "false");
	
	
	}
	
//	else if((can_data.id >= 1280) && (can_data.id <= 1343) && (isESRFull == false))
//	{
//		if(can_data.id == 1280) esr_count = 0;
//		
//		esr_index = can_data.id-1280;
//		check_esr_id[esr_index] = 1;
//		
////		printf("ESR ID : %02X Count : %d\n", can_data.id, esr_count);
////		printf("esr_count : %d\n",lane_count);
//		
//		multican_data.can_msgs.push_back(can_data);
//		
//		if(check_esr_id[esr_index] == 1) esr_count++;
//
//		if(esr_count == 16)
//		{
//			isESRFull =true;
//			esr_count = 0;
//		}
//		
////		printf(isLaneFull ? "true" : "false");
//	
//	
//	}

}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"filter_node");
	ros::NodeHandle nh;

	ros::Subscriber can_sub = nh.subscribe("/received_messages",100,callback);

	ros::Publisher can_pub = nh.advertise<platooning_perception::multican>("/filtered_msgs",100);
	
//	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{

//		if(isRadarFull & isCameraFull & isFusionFull & isLaneFull & isESRFull)
		if(isRadarFull & isCameraFull & isLaneFull)
		{
			int msg_size = 0;
			for(int i=0; i<60; i++){
				printf(" %d",multican_data.can_msgs[i].id);
				
				if((multican_data.can_msgs[i].id >= 257) && (multican_data.can_msgs[i].id <= 272)){msg_size++;}
				else if((multican_data.can_msgs[i].id >= 513) && (multican_data.can_msgs[i].id <= 528)){msg_size++;}
				else if((multican_data.can_msgs[i].id >= 1841) && (multican_data.can_msgs[i].id <= 1850)){msg_size++;}
				else if((multican_data.can_msgs[i].id >= 0x766) && (multican_data.can_msgs[i].id <= 0x769 )){msg_size++;}
//				else if((multican_data.can_msgs[i].id >= 1280) && (multican_data.can_msgs[i].id <= 1343 )){msg_size++;}
			}
			printf("\n multican_data : %d\n",msg_size);
			
			for(int i=0; i<msg_size; i++){
				for(int j = 1+i; j<msg_size; j++){
					if(multican_data.can_msgs[i].id == multican_data.can_msgs[j].id ){
						multican_data.can_msgs[i].id = 0;
					}
				}
			}
			for(int i=0; i<msg_size; i++){
//				printf("%d %d\n",multican_data.can_msgs[i].id,check_id(multican_data.can_msgs[i].id));
				if(check_id(multican_data.can_msgs[i].id))
//				if(multican_data.can_msgs[i].id != 0)
				{
					can_data2.id = multican_data.can_msgs[i].id;
					can_data2.dlc = multican_data.can_msgs[i].dlc;
					can_data2.data = multican_data.can_msgs[i].data;
					multican_data2.can_msgs.push_back(can_data2);
				}
				
			}

			msg_size =0;
			for(int i=0; i<60; i++){
				printf(" %d",multican_data2.can_msgs[i].id);
				if((multican_data2.can_msgs[i].id >= 257) && (multican_data2.can_msgs[i].id <= 272)){msg_size++;}
				else if((multican_data2.can_msgs[i].id >= 513) && (multican_data2.can_msgs[i].id <= 528)){msg_size++;}
				else if((multican_data2.can_msgs[i].id >= 1841) && (multican_data2.can_msgs[i].id <= 1850)){msg_size++;}
				else if((multican_data2.can_msgs[i].id >= 0x766) && (multican_data2.can_msgs[i].id <= 0x769 )){msg_size++;}
//				else if((multican_data2.can_msgs[i].id >= 1280) && (multican_data.can_msgs[i].id <= 1343 )){msg_size++;}
			}

			printf("\n multican_data : %d\n",msg_size);
			
			can_pub.publish(multican_data2);

			for(int i=0; i<70; i++){
				multican_data.can_msgs[i].id = 0;
			}	
			
			double current_time = gettimeafterboot();
			
			double dt = current_time - last_time;
			printf("dt = %lf \n", dt);
			
			last_time = current_time;


			isRadarFull = false;
//			memset(check_radar_id, 0, sizeof(check_radar_id));
			check_radar_id.fill(0);
		
			isCameraFull = false;
			memset(check_camera_id, 0, sizeof(check_camera_id));
			
			isFusionFull = false;
			memset(check_fusion_id, 0, sizeof(check_fusion_id));

			isLaneFull = false;
			memset(check_lane_id, 0, sizeof(check_lane_id));
			
//			isESRFull = false;
//			memset(check_esr_id, 0, sizeof(check_esr_id));
			
			multican_data.can_msgs.clear();
			multican_data2.can_msgs.clear();

			pub_count++;

			printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Radar, Camera, Fusion publish! %d\n",pub_count);
		}


		ros::spinOnce();
	}

	return 0;

}
