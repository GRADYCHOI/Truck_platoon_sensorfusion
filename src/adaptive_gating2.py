#!/usr/bin/env python

import sys
import time
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


import rospy
import roslib
from platooning_perception.msg import multican 

#from visualization_msgs.msg import MarkerArray
#from visualization_msgs.msg import Marker

threshold_x = 60
threshold_y = 80

Gate_size_x = [] # Gate size x of CAM 
Gate_size_y = [] # Gate size y of CAM

C_long = [] # camera_x, longitudinal position of CAM
C_lat = []  # camera_y, lateral position of CAM
R_long = [] # radar_x, longitudinal position of RADAR
R_lat = []  # radar_y, lateral position of RADAR
F_long = []
F_lat = []
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
			data0 = data0.encode("utf-8")
			data1 = data1.encode("utf-8")
			data2 = data2.encode("utf-8")
			data3 = data3.encode("utf-8")
			data4 = data4.encode("utf-8")
			data0 = int(data0,16)
			data1 = int(data1,16)
			data2 = int(data2,16)
			data3 = int(data3,16)
			data4 = int(data4,16)

			if ((data.can_msgs[i].id >= 257) and (data.can_msgs[i].id <= 272)):
				radar_pos_x = float((((data2 & 0x7F)<<8) + data1)*0.01)
				radar_pos_y = float((((data4 & 0x3F)<<8) + data3)*0.01-81.91)
				mov_flag = data4 >> 6;

				if((mov_flag == 1) or (mov_flag == 3)):
					radar_long.append(float(radar_pos_x))
					radar_lat.append(float(radar_pos_y))
				else: continue;


			elif ((data.can_msgs[i].id >= 513) and (data.can_msgs[i].id <= 528)):
				fusion_pos_x = (data0 + ((data1 & 0x7F)<<8))*0.01
				sign = (data3 & 0x10)>>4


				if sign == 0:
					fusion_pos_y = ((data1>>7) + (data2<<1) + ((data3 & 0x1F)<<9))*(-0.01) 
				elif sign == 1: 
					fusion_pos_y = (16384 -((data1>>7)+(data2<<1)+((data3 &0x1F)<<9)))*(0.01) 

				if((fusion_pos_x == 0) and (fusion_pos_y == 0)):
					continue;
				else: 
					fusion_long.append(fusion_pos_x)
					fusion_lat.append(fusion_pos_y)


			elif ((data.can_msgs[i].id >= 1841) and (data.can_msgs[i].id <= 1850)):
				camera_pos_x = (((data2 & 0x03)<<10) + (data1<<2) + (data0>>6))*0.05
				sign = (data3 & 0x20)>>5

				if sign == 0:
					camera_pos_y = (((data3 & 0x3F)<<6) + (data2>>2))*0.05 
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
		FF_long = ffusion_long
		FF_lat = ffusion_lat


#print(C_long)
#		print("C_long:{0:s}".format(C_long))
#		print("C_lat:{0:s}".format(C_lat))
#		print("R_long:{0:s}".format(R_long))
#		print("R_lat:{0:s}".format(R_lat))

    def getGateSize(self, c_data_x, c_data_y, r_data_x, r_data_y, g_x, g_y, f_data_x, f_data_y, ff_data_x, ff_data_y):

	
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
		        g_x.append(0.125 * c_data_x[i] + 5)
		    elif (c_data_x[i] > threshold_x):
		        g_x.append(15)

#			print(g_x)

        for i in range(len(c_data_x)):	
			if ((c_data_x[i] >= 0) and (c_data_x[i] <= threshold_y)):
				g_y.append(0.015 * c_data_x[i] + 1)
			elif (c_data_x[i] > threshold_y):
				g_y.append(2.5)



#self.drawGate(C_long, C_lat, R_long, R_lat, Gate_size_x, Gate_size_y)
        self.drawGate(c_data_x, c_data_y, r_data_x, r_data_y, g_x, g_y, f_data_x, f_data_y, ff_data_x, ff_data_y)

        del Gate_size_x[:] # Gate size x of CAM 
        del Gate_size_y[:] # Gate size y of CAM

        del C_long[:] # camera_x, longitudinal position of CAM
        del C_lat[:]  # camera_y, lateral position of CAM
        del R_long[:] # radar_x, longitudinal position of RADAR
        del R_lat[:]  # radar_y, lateral position of RADAR

#        print("=========end==========")



    def drawGate(self, cam_pos_x, cam_pos_y, radar_pos_x, radar_pos_y, gate_size_x, gate_size_y, fusion_pos_x, fusion_pos_y, ffusion_pos_x, ffusion_pos_y):

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

		for i in range(len(cam_pos_x)):
			ffusion_x.append(0.2*cam_x[i] + 0.8*radar_x[i])

		for i in range(len(cam_pos_y)):
			ffusion_y.append(0.8*cam_y[i] + 0.2*radar_y[i])

		print("len_camlong:{0:s}".format(len(cam_x)))
		print("len_camlat:{0:s}".format(len(cam_y)))
		
		print(ffusion_x)
		print(ffusion_y)

		currentAxis = plt.ion()
		currentAxis = plt.gca()
		
		plt.scatter(cam_x,cam_y)
		plt.scatter(radar_x,radar_y)
		plt.scatter(fusion_x,fusion_y)
		plt.scatter(ffusion_x,ffusion_y)
		
		print('cam pos ',cam_pos_x)
		print('len',len(cam_pos_x))

		print('cam pos y ',cam_pos_y)
		print('len y',len(cam_pos_y))

		print('fusion pos ',fusion_pos_x)
		print('len',len(fusion_pos_x))

		print('fusion pos y ',fusion_pos_y)
		print('len y',len(fusion_pos_y))

		print('gate_size ',gate_size_x)
		print('gate_size len ',len(gate_size_x))

		print('gate_size ',gate_size_y)
		print('gate_size len ',len(gate_size_y))
		print('------------------------')		


		for i in range(len(cam_pos_x)):
			currentAxis.add_patch(Rectangle((cam_pos_x[i] - gate_size_x[i]/2, cam_pos_y[i] - gate_size_y[i]/2), gate_size_x[i], gate_size_y[i], fill = None, alpha = 1))
 

		plt.axis((0,140,-20,20))
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

    HG.getGateSize(C_long,C_lat,R_long,R_lat,Gate_size_x,Gate_size_y,F_long,F_lat,FF_long,FF_lat)
    time.sleep(0.05)

