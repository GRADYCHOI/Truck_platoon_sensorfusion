#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>

//ros
#include <ros/ros.h>
#include <platooning_perception/can.h>
#include <std_msgs/Header.h>

void recv_can();

float pos_x[16],pos_y[16],angle[16];

int sock,count = 0;
struct sockaddr_can addr;//
struct can_filter rfilter[16];
struct can_frame frame;//
int ndx;
struct ifreq ifr;
int ifindex,rd_size;

std_msgs::Header last;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "CANdump");
    ros::NodeHandle nh;
    //ros::Rate loop_rate(200);

    ros::Publisher can_pub = nh.advertise<platooning_perception::can>("/can_msgss",100);

    while(ros::ok()){

        if ( 0 > (sock = socket(PF_CAN, SOCK_RAW, CAN_RAW))) {
            perror("socket");
            return 1;
        }

        rfilter[0].can_id = 0x101;
        rfilter[0].can_mask = CAN_SFF_MASK;
        for(int i = 1; i<16;i++){
            rfilter[i].can_id=rfilter[i-1].can_id+1;
            rfilter[i].can_mask = CAN_SFF_MASK;
        }

        setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        strcpy( ifr.ifr_name, "can0");		// 사용할 CAN 번호
        //strcpy( ifr.ifr_name, "can1");		// 사용할 CAN 번호
        ioctl(sock, SIOCGIFINDEX, &ifr);
        ifindex = ifr.ifr_ifindex;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifindex;
        bind( sock, (struct sockaddr *)&addr, sizeof(addr));


        printf("=================================\n");
        printf("receive start\n");
        rd_size = read( sock, &frame, sizeof(struct can_frame));

        int diff_sec,diff_nsec;
        if(count == 0) {
            last.stamp = ros::Time::now(); 
            //            printf("time set: (%3ld.%06d)\n ",last.stamp.sec, last.stamp.nsec);
            count++;
        }

        diff_sec = ros::Time::now().sec - last.stamp.sec;
        diff_nsec = ros::Time::now().nsec - last.stamp.nsec;

        if(diff_nsec < 0) 
        {
            diff_sec--;
            diff_nsec += 1000000000;
        }

        printf("timestamp : (%3d.%09d)\n ",diff_sec, diff_nsec);

        if (0 > rd_size){
            fprintf(stderr, "read error");
        }
        else if(rd_size < sizeof(struct can_frame)){
            fprintf(stderr, "read: incomplete CAN frame\n");
        }
        else {
            if((frame.can_id >= 257) && (frame.can_id <= 272))
            {
                platooning_perception::can can_data;

                //can_data.header.stamp.sec = diff_sec;
                //can_data.header.stamp.nsec = diff_nsec;
                can_data.id = frame.can_id;
                printf("frame.id : %02X\n",can_data.id);
                can_data.dlc = frame.can_dlc;
                
                for(int ndx = 0; ndx<frame.can_dlc; ndx++){
                    can_data.data[ndx] = frame.data[ndx];

                    printf(" %02X",can_data.data[ndx]);
                }
                printf("\n");

                int flag = can_data.data[4];
                int index;
                flag = flag>>6;

                //if ((flag == 3)||(flag ==1)) 
                if (flag == 3) 
                {
                    index = can_data.id-257;
                    pos_x[index] = (((can_data.data[2] & 0x7F)<<8) + can_data.data[1])*0.01;
                    pos_y[index] = (((can_data.data[4] & 0x3F)<<8) + can_data.data[3])*0.01-81.91;
                    angle[index] = atan2(pos_y[index],pos_x[index])*57.3;

                    printf("frame.id : %02X\n",can_data.id);
                    printf("pos_x : %f\n",pos_x[index]);
                    printf("pos_y : %f\n",pos_y[index]);
                    printf("angle : %f\n",angle[index]);
                    printf("flag : %02X\n",flag);

                    can_pub.publish(can_data);
                }
            }

            printf("=================================\n");

            close( sock);
        }
    }
    //    loop_rate.sleep();


    return 0;

}

