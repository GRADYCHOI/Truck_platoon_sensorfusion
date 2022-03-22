/*
 * canpump.c - reduced CAN data logger using recvmmsg() syscall
 *
 * Copyright (c) 2014 Oliver Hartkopp <socketcan@hartkopp.net>
 *
 * contains portions of candump.c and the recvmmsg(8) man page
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

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


#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */


//extern "C" void sprint_canframe(char *buf , struct canfd_frame *cf, int sep, int maxdlen);


extern "C"
{
#define put_sff_id(buf, id) _put_id(buf, 2, id)
#define put_eff_id(buf, id) _put_id(buf, 7, id)
#define hex_asc_upper_lo(x)	hex_asc_upper[((x) & 0x0F)]
#define hex_asc_upper_hi(x)	hex_asc_upper[((x) & 0xF0) >> 4]


    const char hex_asc_upper[] = "0123456789ABCDEF";
    static inline void _put_id(char *buf, int end_offset, canid_t id)
    {
        while (end_offset >= 0) {
            buf[end_offset--] = hex_asc_upper_lo(id);
            id >>= 4;
        }
    }
    //extern "C" void sprint_canframe(char *buf , struct canfd_frame *cf, int sep, int maxdlen);
    //extern "C" static inline void _put_id(char *buf, int end_offset, canid_t id)
    //extern "C" static inline void put_hex_byte(char *buf, __u8 byte)

    static inline void put_hex_byte(char *buf, __u8 byte)
    {
        buf[0] = hex_asc_upper_hi(byte);
        buf[1] = hex_asc_upper_lo(byte);
    }


    void sprint_canframe(char *buf , struct canfd_frame *cf, int sep, int maxdlen) {

        int i,offset;
        int len = (cf->len > maxdlen) ? maxdlen : cf->len;

        if (cf->can_id & CAN_ERR_FLAG) {
            put_eff_id(buf, cf->can_id & (CAN_ERR_MASK|CAN_ERR_FLAG));
            buf[8] = '#';
            offset = 9;
        } else if (cf->can_id & CAN_EFF_FLAG) {
            put_eff_id(buf, cf->can_id & CAN_EFF_MASK);
            buf[8] = '#';
            offset = 9;
        } else {
            put_sff_id(buf, cf->can_id & CAN_SFF_MASK);
            buf[3] = '#';
            offset = 4;
        }

        if (maxdlen == CAN_MAX_DLEN && cf->can_id & CAN_RTR_FLAG) {
            buf[offset++] = 'R';
            if (cf->len && cf->len <= CAN_MAX_DLC)
                buf[offset++] = hex_asc_upper_lo(cf->len);

            buf[offset] = 0;
            return;
        }

        if (maxdlen == CANFD_MAX_DLEN) {
            buf[offset++] = '#';
            buf[offset++] = hex_asc_upper_lo(cf->flags);
            if (sep && len)
                buf[offset++] = '.';
        }

        for (i = 0; i < len; i++) {
            put_hex_byte(buf + offset, cf->data[i]);
            offset += 2;
            if (sep && (i+1 < len))
                buf[offset++] = '.';
        }

        buf[offset] = 0;
    }
}

int check_radar_id[16] = {0,};
int check_camera_id[10] = {0,};
int isRadarFull = 0;
int isCameraFull = 0;
int radar_count = 0;
int camera_count = 0;

std_msgs::Header last;

static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */

int count = 0;

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}

#define VLEN 20

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CAN");
    ros::NodeHandle nh;

    ros::Publisher can_pub = nh.advertise<platooning_perception::multican>("/can_msgs",100);

	int s; /* can raw socket */
	int enable_sockopt = 1;
	struct sockaddr_can addr;
	struct sockaddr_can addrs[VLEN];
	struct ifreq ifr;

	char ctrlmsgs[VLEN][CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct cmsghdr *cmsg;

	struct canfd_frame frames[VLEN];
	struct iovec iovecs[VLEN];
	struct mmsghdr mmsghdrs[VLEN];

	char buf[CL_CFSZ]; /* max length */
    char id[3];
    char data[8][3];
    int nframes, maxdlen, idx, i;
    struct timeval tv = { 0, 0 };


	platooning_perception::multican msg;

	while(ros::ok()) {

			/* open socket */
			if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
					perror("socket");
					return 1;
			}

			addr.can_family = AF_CAN;

			strcpy( ifr.ifr_name, "can0");		// 사용할 CAN 번호

			if (strcmp(ANYDEV, ifr.ifr_name)) {
					if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
							perror("SIOCGIFINDEX");
							exit(1);
					}
					addr.can_ifindex = ifr.ifr_ifindex;
			} else
					addr.can_ifindex = 0; /* any can interface */

			/* try to switch the socket into CAN FD mode */
			setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_sockopt, sizeof(enable_sockopt));

			if (setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &enable_sockopt, sizeof(enable_sockopt)) < 0) {
					perror("setsockopt SO_TIMESTAMP");
					return 1;
			}

			if (setsockopt(s, SOL_SOCKET, SO_RXQ_OVFL, &enable_sockopt, sizeof(enable_sockopt)) < 0) {
					perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
					/* continue without dropmonitor */
			}

			if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
					perror("bind");
					return 1;
			}

			/* these settings are static and can be held out of the hot path */
			memset(frames, 0, sizeof(frames));
			memset(addrs, 0, sizeof(addrs));
			memset(iovecs, 0, sizeof(iovecs));
			memset(mmsghdrs, 0, sizeof(mmsghdrs));
			for (i = 0; i < VLEN; i++) {
					iovecs[i].iov_base = &frames[i];
					//		iovecs[i].iov_len = BUFSIZE;
					mmsghdrs[i].msg_hdr.msg_name= &addrs[i];
					mmsghdrs[i].msg_hdr.msg_iov = &iovecs[i];
					mmsghdrs[i].msg_hdr.msg_iovlen = 1;
					mmsghdrs[i].msg_hdr.msg_control = &ctrlmsgs[i];
			}

			nframes = VLEN;


			int num_msg = 0;

			/* these settings may be modified by recvmsg() */
			for (i = 0; i < nframes; i++) {
					iovecs[i].iov_len = sizeof(frames[0]);
					mmsghdrs[i].msg_hdr.msg_namelen = sizeof(addrs[0]);
					mmsghdrs[i].msg_hdr.msg_controllen = sizeof(ctrlmsgs[0]);
					mmsghdrs[i].msg_hdr.msg_flags = 0;
			}

			printf("=================================\n");
			printf("receive start\n");

			nframes = recvmmsg(s, mmsghdrs, VLEN, MSG_WAITFORONE, NULL);

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

			if (nframes < 0) {
					perror("recvmmsg()");
					return 1;
			}

			for (i = 0; i < nframes; i++) {


					if ((size_t)mmsghdrs[i].msg_len == CAN_MTU)
							maxdlen = CAN_MAX_DLEN;
					else if ((size_t)mmsghdrs[i].msg_len == CANFD_MTU)
							maxdlen = CANFD_MAX_DLEN;
					else {
							fprintf(stderr, "read: incomplete CAN frame\n");
							return 1;
					}

					idx = idx2dindex(addrs[i].can_ifindex, s);

					/* print CAN frame in log file style to stdout */
					sprint_canframe(buf, &frames[i], 0, maxdlen);

					//data send

					platooning_perception::can can_data;

					memcpy(id, buf, 3);
					id[3] = 0;
					//printf("ID before : %s\n",id);
					can_data.id = strtol(id,NULL,16);
					//printf("ID : %02X\n", can_data.id);
					can_data.dlc = 8;

					for(int k=0; k<8; k++){
							memcpy(&data[k], &buf[4+2*k], 2);
							data[k][2] = 0;
							//                printf(" %s",&data[k]);

							can_data.data[k]=strtol(data[k],NULL,16);
							// frame_data[k]=strtol(&data[k],0,16);
							//printf(" %02X",can_data.data[k]);
					}
					printf("\n");

					int flag = can_data.data[4];
					int radar_index,camera_index;
					flag = flag>>6;

					if((can_data.id >= 257) && (can_data.id <= 272)) 
					{
							radar_index = can_data.id-257;
							check_radar_id[radar_index] = 1;
							printf("Radar ID : %02X\n", can_data.id);

							msg.header.stamp.sec = diff_sec;
							msg.header.stamp.nsec = diff_nsec;
							msg.can_msgs.push_back(can_data);

							for (i=0; i<=15; i++){
									if(check_radar_id[i] == 1) radar_count++;
									// else if(check_radar_id[i] == 0) break;
							}

							printf("Radar count : %d\n", radar_count);

							if(radar_count == 16) isRadarFull = 1;

							radar_count = 0;
					}

					else if((can_data.id >= 1841) && (can_data.id <= 1850))
					{
							camera_index = can_data.id - 1841;
							check_camera_id[camera_index] = 1;
							printf("Camera ID : %02X\n", can_data.id);

							for(int j=0; j<8; j++) printf(" %02X",can_data.data[j]);
							printf("\n");

							msg.header.stamp.sec = diff_sec;
							msg.header.stamp.nsec = diff_nsec;
							msg.can_msgs.push_back(can_data);

							for (int k=0; k<=9; k++){
									if(check_camera_id[k] == 1) camera_count++;
									//  else if(check_camera_id[k] == 0) break;
							}

							printf("Camera count : %d\n", camera_count);

							if(camera_count == 10) isCameraFull = 1;

							camera_count = 0;
					}

					//else break;


					fflush(stdout);

			}

			if(isRadarFull & isCameraFull){
					printf("puablish\n");

					can_pub.publish(msg); //publish message

					check_camera_id[10] = {0,};
					isCameraFull = 0;

					check_radar_id[16] = {0,};
					isRadarFull = 0;

					msg.can_msgs.clear();

			}



			printf("=================================\n");
			close(s);
	}


	return 0;
}
