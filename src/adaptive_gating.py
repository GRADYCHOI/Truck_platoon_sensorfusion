#!/usr/bin/env python

import sys
import time
import numpy as np
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


import rospy
import roslib
from platooning_perception.msg import multican 

#from visualization_msgs.msg import MarkerArray
#from visualization_msgs.msg import Marker

threshold_x = 60
threshold_y = 80

CGate_size_x = [] # Gate size x of CAM 
CGate_size_y = [] # Gate size y of CAM

RGate_size_x = [] # Gate size x of RADAR 
RGate_size_y = [] # Gate size y of RADAR



C_long = [] # camera_x, longitudinal position of CAM
C_lat = []  # camera_y, lateral position of CAM
R_long = [] # radar_x, longitudinal position of RADAR
R_lat = []  # radar_y, lateral position of RADAR
F_long = []
F_lat = []

MINMAX_data_x = []
MINMAX_data_y = []

FF_long = []
FF_lat = []


class Adaptive_gating:

    def __init__(self):
        self._can_sub = rospy.Subscriber("/filtered_msgs",multican,self.canCallback)

    def canCallback(self,data):
   
		global C_long, C_lat, R_long, R_lat, F_long, F_lat, FF_long, FF_lat

		cam_long=[]
		cam_lat=[]
		radar_long=[]
		radar_lat=[]
		fusion_long=[]
		fusion_lat=[]
		fusion_meas=[]
		ffusion_long=[]
		ffusion_lat=[]

#		print("=============start============")
#		print(len(data.can_msgs))

		for i in range(len(data.can_msgs)):

			data0 = data.can_msgs[i].data[0].encode("hex")
			data1 = data.can_msgs[i].data[1].encode("hex")
			data2 = data.can_msgs[i].data[2].encode("hex")
			data3 = data.can_msgs[i].data[3].encode("hex")
			data4 = data.can_msgs[i].data[4].encode("hex")
			data7 = data.can_msgs[i].data[7].encode("hex")
			data0 = data0.encode("utf-8")
			data1 = data1.encode("utf-8")
			data2 = data2.encode("utf-8")
			data3 = data3.encode("utf-8")
			data4 = data4.encode("utf-8")
			data7 = data7.encode("utf-8")
			data0 = int(data0,16)
			data1 = int(data1,16)
			data2 = int(data2,16)
			data3 = int(data3,16)
			data4 = int(data4,16)
			data7 = int(data7,16)

			if ((data.can_msgs[i].id >= 257) and (data.can_msgs[i].id <= 272)):
				radar_pos_x = float((((data2 & 0x7F)<<8) + data1)*0.01 + 0.266)
#				radar_pos_y = float((((data4 & 0x3F)<<8) + data3)*(-0.01) + 81.91 - 0.24)
				radar_pos_y = float((((data4 & 0x3F)<<8) + data3)*(0.01) - 81.91 - 0.24)
				if(radar_pos_x > 150 ): continue;
				mov_flag = data4 >> 6;

				if((mov_flag == 1) or (mov_flag == 3)):
					radar_long.append(float(radar_pos_x))
					radar_lat.append(float(radar_pos_y))
				else: continue;
#				radar_long.append(float(radar_pos_x))
#				radar_lat.append(float(radar_pos_y))


			elif ((data.can_msgs[i].id >= 513) and (data.can_msgs[i].id <= 528)):
				fusion_pos_x = (data0 + ((data1 & 0x7F)<<8))*0.01
				sign = (data3 & 0x10)>>4
				meas = (data7>>6)

				if sign == 0:
					fusion_pos_y = ((data1>>7) + (data2<<1) + ((data3 & 0x1F)<<9))*(-0.01) 
				elif sign == 1: 
					fusion_pos_y = (16384 -((data1>>7)+(data2<<1)+((data3 &0x1F)<<9)))*(0.01) 

				if((fusion_pos_x == 0) and (fusion_pos_y == 0)):
					continue;
				else: 
					fusion_long.append(fusion_pos_x)
					fusion_lat.append(fusion_pos_y)
				fusion_meas.append(meas)




			elif ((data.can_msgs[i].id >= 1841) and (data.can_msgs[i].id <= 1850)):
#				camera_pos_xx = (((data2 & 0x03)<<10) + (data1<<2) + (data0>>6))

				camera_pos_x = (((data2 & 0x03)<<10) + (data1<<2) + (data0>>6))*0.05
				if(camera_pos_x > 150 ): continue;
				
				sign = (data3 & 0x20)>>5

				if sign == 0:
					camera_pos_y = (((data3 & 0x3F)<<6) + (data2>>2))*(0.05) 
				elif sign == 1: 
					camera_pos_y = (4096 - (((data3 & 0x3F)<<6) + (data2>>2)))*(-0.05) 

				if((camera_pos_x == 0) and (camera_pos_y == 0)):
					continue;
				else: 
					cam_long.append(camera_pos_x)
					cam_lat.append(camera_pos_y)

#		print("=============end============")
		C_long = cam_long
		C_lat = cam_lat
		R_long = radar_long
		R_lat = radar_lat
		F_long = fusion_long
		F_lat = fusion_lat
#		print(fusion_meas)

    def Fusion(self, cam_longi, cam_late, radar_longi, radar_late, cg_x, cg_y, rg_x, rg_y ):
		

		lc = len(cam_longi)
		lr = len(radar_longi)
		l_min = min(lc,lr)
		l_max = max(lc,lr)
		print("cam, radar ", lc, lr)

		if((lc != 0) and (lr != 0)):
			for n in range(l_max):
				for m in range(l_min):
					if(lc < lr):
						j = n
						i = m
					else:
						i = n
						j = m 

					iou_check = False

					cx_max = (cam_longi[i] + cg_x[i]/2)
					cx_min = (cam_longi[i] - cg_x[i]/2)
					cy_max = (cam_late[i] + cg_y[i]/2)
					cy_min = (cam_late[i] - cg_y[i]/2)
				
					rx_max = (radar_longi[j] + rg_x[j]/2)
					rx_min = (radar_longi[j] - rg_x[j]/2)
					ry_max = (radar_late[j] + rg_y[j]/2)
					ry_min = (radar_late[j] - rg_y[j]/2)
			
					x_min = ( max(rx_min,cx_min))
					x_max = ( min(rx_max,cx_max))
					y_min = ( max(ry_min,cy_min))
					y_max = ( min(ry_max,cy_max))

					iou_x = (x_max - x_min)
					iou_y = (y_max - y_min)

					print("x_max", x_max)
					print("x_min", x_min)
					print("y_max", y_max)
					print("y_min", y_min)
				
					print(iou_x)
					print(iou_y)
					if((iou_x > 0) and (iou_y > 0)):
						iou_check = True
					print(iou_check)
					print("n , m ",n,m)
					print("i , j ",i,j)
					print("=====================\n")	

					if (iou_check == True): # meas = 3_both
						if((cam_longi[i] < 140) and (radar_longi[j] < 140)):
							MINMAX_data_x.append(cam_longi[i]*0.365 + radar_longi[j]*0.635)
							MINMAX_data_y.append(cam_late[i]*0.98 + radar_late[j]*0.02)
							print("fusion! \n")
				
					elif((iou_check == False) and (lc <= lr)):
#						for radar_only in range(lr-1,lc-1,-1):
						if((radar_longi[i] < 150) and (radar_late[i] > -13) and (radar_late[i] < 13)): # meas = 1_radar
							MINMAX_data_x.append(radar_longi[i])
							MINMAX_data_y.append(radar_late[i])
							print("only_radar_fusion! \n")
					elif((iou_check == False) and (lc >= lr)):
#						for cam_only in range(lc-1,lr-1,-1):
						if((cam_longi[i] < 140) and (cam_late[i] > -15) and (cam_late[i] < 15)): # meas = 2_camera
							MINMAX_data_x.append(cam_longi[i])
							MINMAX_data_y.append(cam_late[i])
							print("only_camera_fusion! \n")

		elif(lc == 0):
			for i in range(lr):
				MINMAX_data_x.append(radar_longi[i])
				MINMAX_data_y.append(radar_late[i])
				print("only_radar_fusion! \n")
		
		elif(lr == 0):	
			for i in range(lc):
				MINMAX_data_x.append(cam_longi[i])
				MINMAX_data_y.append(cam_late[i])
				print("only_camera_fusion! \n")
			

#	    print(MINMAX_data_x)
#	    print(MINMAX_data_y)

#        FF_long = ffusion_long
#		FF_lat = ffusion_lat


#print(C_long)
#		print("C_long:{0:s}".format(C_long))
#		print("C_lat:{0:s}".format(C_lat))
#		print("R_long:{0:s}".format(R_long))
#		print("R_lat:{0:s}".format(R_lat))




	


    def getGateSize(self, c_data_x, c_data_y, r_data_x, r_data_y, cg_x, cg_y, rg_x, rg_y, f_data_x, f_data_y, ff_data_x, ff_data_y):

	
	'''
	marker = Marker()
        marker.header.frame_id = "Adaptive_gating"
	marker.header.stamp = rospy.Time.now()
        marker.ns = "bezier"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.a = 0.2
	marker.color.r = 0.2
	marker.color.g = 0.2
	marker.color.b = 0.2
	marker.pos.orientation.w = 1.0
	marker.pos.position.x

	'''
#        print("=========start==========")
        for i in range(len(c_data_x)):
		    if ((c_data_x[i] >= 0) and (c_data_x[i] <= threshold_x)):
		        cg_x.append(0.125 * c_data_x[i] + 5)
		    elif (c_data_x[i] > threshold_x):
		        cg_x.append(15)

        for i in range(len(c_data_x)):	
			if ((c_data_x[i] >= 0) and (c_data_x[i] <= threshold_y)):
				cg_y.append(0.015 * c_data_x[i] + 1)
			elif (c_data_x[i] > threshold_y):
				cg_y.append(2.5)

        for i in range(len(r_data_x)):
		    if ((r_data_x[i] >= 0) and (r_data_x[i] <= threshold_x)):
		        rg_x.append(0.05 * r_data_x[i] + 2)
		    elif (r_data_x[i] > threshold_x):
		        rg_x.append(5)

        for i in range(len(r_data_x)):	
			if ((r_data_x[i] >= 0) and (r_data_x[i] <= threshold_y)):
				rg_y.append(0.0375 * r_data_x[i] + 2)
			elif (r_data_x[i] > threshold_y):
				rg_y.append(5)

    
        self.Fusion(c_data_x, c_data_y, r_data_x, r_data_y, cg_x, cg_y, rg_x, rg_y )
        self.drawGate(c_data_x, c_data_y, r_data_x, r_data_y, cg_x, cg_y, rg_x, rg_y, f_data_x, f_data_y, MINMAX_data_x, MINMAX_data_y)

        del CGate_size_x[:] # Gate size x of CAM 
        del CGate_size_y[:] # Gate size y of CAM
        del RGate_size_x[:] # Gate size x of CAM 
        del RGate_size_y[:] # Gate size y of CAM
        del MINMAX_data_x[:]
        del MINMAX_data_y[:]


        del C_long[:] # camera_x, longitudinal position of CAM
        del C_lat[:]  # camera_y, lateral position of CAM
        del R_long[:] # radar_x, longitudinal position of RADAR
        del R_lat[:]  # radar_y, lateral position of RADAR
        del F_long[:]
        del F_lat[:]

#        print("=========end==========")



    def drawGate(self, cam_pos_x, cam_pos_y, radar_pos_x, radar_pos_y, cgate_size_x, cgate_size_y, rgate_size_x, rgate_size_y, fusion_pos_x, fusion_pos_y, ffusion_pos_x, ffusion_pos_y):

#		print(cam_pos_x)

		plt.clf() # clear figure
		
		cam_x = cam_pos_x 
		radar_x = radar_pos_x 
		fusion_x = fusion_pos_x
		ffusion_x = ffusion_pos_x

		cam_y = cam_pos_y
		radar_y = radar_pos_y 
		fusion_y = fusion_pos_y
		ffusion_y = ffusion_pos_y


#		print("len_camlong:{0:d}".format(len(cam_x)))
#		print("len_camlat:{0:d}".format(len(cam_y)))
		
#		print(ffusion_x)
#		print(ffusion_y)

		currentAxis = plt.ion()
		currentAxis = plt.gca()
		
#		c = plt.scatter(cam_x,cam_y, s=7**2, alpha=0.8)
#		r = plt.scatter(radar_x,radar_y, s=7**2, alpha=0.8)
#		f = plt.scatter(fusion_x, fusion_y, s=10**2, c='yellow', marker='x',alpha=0.8)
#		ff = plt.scatter(ffusion_x, ffusion_y, s=10**2, c='red', marker='x',alpha=0.8)
		
		c = plt.scatter(cam_y,cam_x, s=7**2, alpha=0.8)
		r = plt.scatter(radar_y,radar_x, s=7**2, alpha=0.8)
		f = plt.scatter(fusion_y, fusion_x, s=10**2, c='yellow', marker='x',alpha=0.8)
		ff = plt.scatter(ffusion_y, ffusion_x, s=10**2, c='red', marker='x',alpha=0.8)

		print('cam pos x ',cam_pos_x)
		print('len x',len(cam_pos_x))

		print('cam pos y ',cam_pos_y)
		print('len y',len(cam_pos_y))
#
		print('radar pos x',radar_pos_x)
		print('len x',len(radar_pos_x))
#
		print('radar pos y ',radar_pos_y)
		print('len y',len(radar_pos_y))
#		
#		print('fusion pos ',fusion_pos_x)
#		print('len',len(fusion_pos_x))
#
#		print('fusion pos y ',fusion_pos_y)
#		print('len y',len(fusion_pos_y))
#
#		print('fusion pos ',ffusion_pos_x)
#		print('len',len(ffusion_pos_x))
#
#		print('fusion pos y ',ffusion_pos_y)
#		print('len y',len(ffusion_pos_y))
#		
		print('------------------------')		


#		for i in range(len(cam_pos_x)):
#			currentAxis.add_patch(Rectangle((cam_pos_x[i] - cgate_size_x[i]/2, cam_pos_y[i] - cgate_size_y[i]/2), cgate_size_x[i], cgate_size_y[i], fill = None, alpha = 1))
#
#		for i in range(len(radar_pos_x)):
#			currentAxis.add_patch(Rectangle((radar_pos_x[i] - rgate_size_x[i]/2, radar_pos_y[i] - rgate_size_y[i]/2), rgate_size_x[i], rgate_size_y[i], fill = None, alpha = 1))

		for i in range(len(cam_pos_x)):
			currentAxis.add_patch(Rectangle((cam_pos_y[i] - cgate_size_y[i]/2, cam_pos_x[i] - cgate_size_x[i]/2), cgate_size_y[i], cgate_size_x[i], fill = None, alpha = 1))
 
		for i in range(len(radar_pos_x)):
			currentAxis.add_patch(Rectangle((radar_pos_y[i] - rgate_size_y[i]/2, radar_pos_x[i] - rgate_size_x[i]/2), rgate_size_y[i], rgate_size_x[i], fill = None, alpha = 1))

		
		currentAxis.patch.set_facecolor('#ababab')
		plt.legend((c,r,f,ff),('Mobileye','Radar','LG Fusion','Our Fusion'),loc=1)
#		plt.xlabel('Longitudinal Direction, X')
#		plt.ylabel('Traverse Direction, Y')
#		plt.axhline(y=2.5,color='w', linestyle = '--',lw=6)
#		plt.axhline(y=-2.5,color='w', linestyle = '--',lw=6)
#		plt.axis((0,140,-12,12))
		
		plt.ylabel('Longitudinal Direction, X')
		plt.xlabel('Traverse Direction, Y')
		plt.axvline(x=2,color='w', linestyle = '--',lw=6)
		plt.axvline(x=-2,color='w', linestyle = '--',lw=6)
		plt.axvline(x=4.5,color='w', linestyle = '--',lw=6)
		plt.axvline(x=-4.5,color='w', linestyle = '--',lw=6)
		plt.axis((-12,12,0,140)) # (0,200,-25,25)
		
		plt.pause(0.01)
		plt.show()

HG = Adaptive_gating()

def main():

    rospy.init_node("adaptive_gating",anonymous=True)

    r = rospy.Rate(5)
    plt.figure()
    try:
        r.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

while not rospy.is_shutdown():

    HG.getGateSize(C_long,C_lat,R_long,R_lat,CGate_size_x,CGate_size_y,RGate_size_x,RGate_size_y,F_long,F_lat,FF_long,FF_lat)
    time.sleep(0.05)

