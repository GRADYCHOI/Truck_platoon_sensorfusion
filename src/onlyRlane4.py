#!/usr/bin/env python

import numpy as np
import cv2
import pickle
import glob
#import matplotlib.pyplot as plt
from ipywidgets import interact, interactive, fixed
from moviepy.editor import VideoFileClip
from IPython.display import HTML
import time

### for using ROS
import rospy
import roslib
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import UInt32 
from platooning_perception.msg import Range 
from platooning_perception.msg import lane 
from platooning_perception.msg import multican 
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo 
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

def unwarp(img, src, dst):
    h,w = img.shape[:2]
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, (w,h), flags=cv2.INTER_LINEAR)
    return warped, M, Minv

def abs_sobel_thresh(img, orient='x', thresh_min=10, thresh_max=255):
    gray = (cv2.cvtColor(img, cv2.COLOR_RGB2Lab))[:,:,0]
    sobel = cv2.Sobel(gray, cv2.CV_64F, orient=='x', orient=='y')
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1
    binary_output = sxbinary 
    return binary_output 

def hls_lthresh(img, thresh=(220, 255)): #200
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    hls_l = hls[:,:,1]
    hls_l = hls_l*(255/np.max(hls_l))
    binary_output = np.zeros_like(hls_l)
    binary_output[(hls_l > thresh[0]) & (hls_l <= thresh[1])] = 1
    
    return binary_output

def pipeline(img):
    img_sobelAbs = abs_sobel_thresh(img,'x',25)
    img_LThresh = hls_lthresh(img)
    combined = np.zeros_like(img_LThresh)
    combined[(img_LThresh == 1) | (img_sobelAbs ==1)] = 1
        
    return combined, Minv

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def roi_range(distance):
    if distance < 80 : 
        ROI_y_point = 80
        print("case1")
    else:
        distance = (distance//10-1)*10+5
        ROI_y_point = distance
        print("case2")
    print(ROI_y_point)
    ROI_L_x_point = ((700 - ROI_y_point) * 0.6)+200
    ROI_R_x_point = ((ROI_y_point - 700) * 0.6)+980

    return ROI_L_x_point, ROI_R_x_point, ROI_y_point

def sliding_window_polyfit(img, last_leftx_base, last_rightx_base, last_left_fit, check):
    histogram = np.sum(img[:,:], axis=0)

    origin_histogram = histogram
    
    # Check last bases was initialized
    if last_leftx_base != 0 or last_rightx_base != 0: 
        
        distrib_width = 200
        sigma = distrib_width / 12   
        
        leftx_range = np.arange(last_leftx_base - distrib_width/2, last_leftx_base + distrib_width/2, dtype=int)
        rightx_range = np.arange(last_rightx_base - distrib_width/2, last_rightx_base + distrib_width/2, dtype=int)

        weight_distrib = np.zeros(img.shape[1])
        for i in range(img.shape[1]):
            if i in leftx_range:
                weight_distrib[i] = gaussian(i, last_leftx_base, sigma)
            elif i in rightx_range:
                weight_distrib[i] = gaussian(i, last_rightx_base, sigma)

        histogram = np.multiply(histogram, weight_distrib)

    midpoint = np.int(histogram.shape[0]//2)
    quarter_point = np.int(midpoint//2)

    left_offset = 80
    leftx_base = np.argmax(histogram[quarter_point-3*left_offset:quarter_point-left_offset]) + quarter_point - 3*left_offset
    #print("leftx_base: {0}".format(leftx_base))

    rightx_base = np.argmax(histogram[midpoint+quarter_point:(midpoint+quarter_point + 3*left_offset)]) + midpoint+quarter_point
    
    nwindows = 10
    window_height = np.int(img.shape[0]/nwindows)
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 50
    minpix = 400
    goodpix = 1100
    maxpix = 3100
    dist = 890
    left_lane_inds = []
    right_lane_inds = []
    rectangle_data = []
    
    if check:
        count = 7
    else:
        count = 0   

    for window in range(nwindows):
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        
        
        win_xleft_low = leftx_current - margin 
        win_xleft_high = leftx_current + margin 
        
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        rectangle_data.append((win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high))
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        #print("point: {0}".format(good_left_inds))
        #print("point_number: {0}".format(len(good_left_inds)))
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if maxpix > len(good_left_inds) > goodpix:
                count += 1
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    leftx_ = nonzerox[right_lane_inds]-dist
    lefty_ = nonzeroy[right_lane_inds] 
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    left_fit, right_fit = (None, None)
    
    if len(leftx) != 0:
        if count > 4:
            left_fit = np.polyfit(lefty, leftx, 2)
            count = 0
            
        else:
            left_fit = np.polyfit(lefty_, leftx_, 2)
    #print("count: {0}".format(count))
            count = 0    
    if len(rightx) != 0:
        right_fit = np.polyfit(righty, rightx, 2) 
    '''
    if len(rightx) != 0:
        right_fit = np.polyfit(righty, rightx, 2) 
        
    if len(leftx) != 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    '''       
    visualization_data = (rectangle_data, histogram, origin_histogram)
    
    return left_fit, right_fit, left_lane_inds, right_lane_inds, visualization_data, leftx_base, rightx_base, last_left_fit

def calc_curv_rad_and_center_dist(bin_img, l_fit, r_fit, l_lane_inds, r_lane_inds):
    ym_per_pix = 3.048/100 
    xm_per_pix = 3.7/378 
    left_curverad, right_curverad, center_dist = (0, 0, 0)
    h = bin_img.shape[0]
    ploty = np.linspace(0, h-1, h)
    y_eval = np.max(ploty)
  
    
    nonzero = bin_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx = nonzerox[l_lane_inds]
    lefty = nonzeroy[l_lane_inds] 
    rightx = nonzerox[r_lane_inds]
    righty = nonzeroy[r_lane_inds]
    
    if len(leftx) != 0 and len(rightx) != 0:
        left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    
    
    if r_fit is not None and l_fit is not None:
        car_position = bin_img.shape[1]/2
        l_fit_x_int = l_fit[0]*h**2 + l_fit[1]*h + l_fit[2]
        r_fit_x_int = r_fit[0]*h**2 + r_fit[1]*h + r_fit[2]
        lane_center_position = (r_fit_x_int + l_fit_x_int) /2
        center_dist = (car_position - lane_center_position) * xm_per_pix
        #print(center_dist)


    return left_curverad, right_curverad, center_dist

def draw_lane(original_img, binary_img, l_fit, r_fit, Minv):
    new_img = np.copy(original_img)
    if l_fit is None or r_fit is None:
        return original_img
    warp_zero = np.zeros_like(binary_img).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    h,w = binary_img.shape
    ploty = np.linspace(0, h-1, num=h)
    left_fitx = l_fit[0]*ploty**2 + l_fit[1]*ploty + l_fit[2]
    right_fitx = r_fit[0]*ploty**2 + r_fit[1]*ploty + r_fit[2]

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    cv2.fillPoly(color_warp, np.int_([pts]), (0,70, 0))
    cv2.polylines(color_warp, np.int32([pts_left]), isClosed=False, color=(0,255,0), thickness=30)
    cv2.polylines(color_warp, np.int32([pts_right]), isClosed=False, color=(0,255,0), thickness=30)
    # edit for Using ROS
    pts_left = cv2.perspectiveTransform(pts_left, Minv)

    pts_left = np.reshape(pts_left,(-1,2))
    print(len(pts_left))
     
    lane_data.left.x = []
    lane_data.left.y = []

    for i in range(720):
        lane_data.left.x.append(pts_left[i][0])
        lane_data.left.y.append(pts_left[i][1])
     

    pts_right = cv2.perspectiveTransform(pts_right, Minv)

    pts_right = np.reshape(pts_right,(-1,2))

    lane_data.right.x = []
    lane_data.right.y = []
    
    for i in range(720):
        lane_data.right.x.append(pts_right[i][0])
        lane_data.right.y.append(pts_right[i][1])
    # 

    newwarp = cv2.warpPerspective(color_warp, Minv, (w, h)) 
    result = cv2.addWeighted(new_img, 1, newwarp, 0.5, 0)

    return result

# Initialization
last_leftx_base = 0
last_rightx_base = 0

margin = 80
rightx_first = 0
rightx_diff = 0
last_left_fit = []
check = True

target_id = 0;
distance = 0;
check_recv = 0;

# edit for Using ROS
class image_converter:

    def __init__(self):
    	self._bridge = CvBridge()
#self._image_sub = rospy.Subscriber('/videofile/image_raw',Image,self.callback)
        self._image_sub = rospy.Subscriber('/videofile/image_raw',Image,self.callback)
#      	self._image_sub = rospy.Subscriber('/usb_cam/image_raw',Image,self.callback)
        self._time_sub = rospy.Subscriber('/videofile/camera_info',CameraInfo,self.Timecallback)
        self._can_sub = rospy.Subscriber('/can_msgs',multican,self.cancallback)
        self._target_sub = rospy.Subscriber('/target',UInt32,self.targetcallback)
        self.image_pub = rospy.Publisher("/lane_image",Image)
#        self.image_raw_pub = rospy.Publisher("/lane_image_raw",Image)
        self.range_pub = rospy.Publisher("/lane_range",lane,queue_size=100)

    def image_send(self,cv_image):
        try:
            self.image_pub.publish(self._bridge.cv2_to_imgmsg(cv_image,"rgb8"))
	    lane_data.header.stamp = rospy.Time.now()

        except CvBridgeError as e:
            print(e)

    def Timecallback(self,data):
        global get_time_sec,get_time_nsec
        get_time_sec = data.header.stamp.secs
        get_time_nsec = data.header.stamp.nsecs

    def callback(self,data):
    	try:
            global cv_image
    	    cv_image = self._bridge.imgmsg_to_cv2(data,"rgb8")
    	except CvBridgeError as e:
    	    print (e)
    
    def cancallback(self,data):
        global target_id,distance
   
        for i in range(len(data.can_msgs)):

            data1 = data.can_msgs[i].data[1].encode("hex")
            data2 = data.can_msgs[i].data[2].encode("hex")
            data3 = data.can_msgs[i].data[3].encode("hex")
            data4 = data.can_msgs[i].data[4].encode("hex")
            data1 = data1.encode("utf-8")
            data2 = data2.encode("utf-8")
            data3 = data3.encode("utf-8")
            data4 = data4.encode("utf-8")
            data1 = int(data1,16)
            data2 = int(data2,16)
            data3 = int(data3,16)
            data4 = int(data4,16)

            if(target_id == data.can_msgs[i].id):
                distance = (((data2 & 0x7F)<<8) + data1)*0.01

    def targetcallback(self,data):
        global target_id,distance 
#target_id = data.data;
        distance = data.data;
        check_recv = 1


def pub():
    ic.range_pub.publish(lane_data)




       
ic = image_converter()

#lane_left=Float32MultiArray()
lane_data=lane()
can_data = multican()

def main():
    rospy.init_node('platooning_perceptionector')
    global range_coordinate
#    timer = rospy.Timer(rospy.Duration(1.0), timer_callback)
    r = rospy.Rate(10)
    try:
    	r.sleep()
#    	rospy.spin()

    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

	   
if __name__ == '__main__':
    main()
#

while not rospy.is_shutdown():
    # Start time
    start = time.time()

    check_recv=0;

    global cv_image

    exampleImg = cv_image

    exampleImg = cv2.resize(exampleImg, dsize=(1280, 720), interpolation=cv2.INTER_AREA)
    exampleImg = cv2.cvtColor(exampleImg, cv2.COLOR_BGR2RGB)
        
    h,w = exampleImg.shape[:2]
        
    ROI_L_x_point, ROI_R_x_point, ROI_y_point = roi_range(distance)

    src = np.float32([(ROI_L_x_point ,ROI_y_point),  
                    (ROI_R_x_point ,ROI_y_point), 
                    (200 ,700), 
                    (980 ,700)])
    dst = np.float32([(250,0),
                    (w-250,0),
                    (250,h),
                    (w-250,h)])
    '''
    src = np.float32([(470 + rightx_diff,180),
                    (710 + rightx_diff,180), 
                    (100 + rightx_diff,630), 
                    (1080 + rightx_diff,630)])
    dst = np.float32([(250,0),
                    (w-250,0),
                    (250,h),
                    (w-250,h)])
    '''
    exampleImg_unwarp, M, Minv = unwarp(exampleImg, src, dst)  
    #exampleImg_bin = cv2.Canny(exampleImg_unwarp, 10,70)
    exampleImg_bin, Minv = pipeline(exampleImg_unwarp)
    
    left_fit, right_fit, left_lane_inds, right_lane_inds, visualization_data, last_leftx_base, last_rightx_base, last_left_fit = sliding_window_polyfit(exampleImg_bin, last_leftx_base, last_rightx_base, last_left_fit, check)
    
    check = False
    
    if rightx_first == 0:
        rightx_first = last_rightx_base
        
    rightx_diff = last_rightx_base - rightx_first
    rightx_diff = rightx_diff / 2.5
        
    #print("rightx_diff: {0}".format(rightx_diff))

    # End time
    end = time.time()

    rectangles = visualization_data[0]
    histogram = visualization_data[1]

    out_img = np.uint8(np.dstack((exampleImg_bin, exampleImg_bin, exampleImg_bin))*255)
    window_img = np.zeros_like(out_img)
    ploty = np.linspace(0, exampleImg_bin.shape[0]-1, exampleImg_bin.shape[0] )
    if not isinstance(left_fit, type(None)) and not isinstance(right_fit, type(None)):
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    for rect in rectangles:
        cv2.rectangle(out_img,(rect[2],rect[0]),(rect[3],rect[1]),(0,255,0), 2) 
        cv2.rectangle(out_img,(rect[4],rect[0]),(rect[5],rect[1]),(0,255,0), 2) 
    nonzero = exampleImg_bin.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [0, 255, 255]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 255, 255]
    
    
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    #cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
    #cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
   
    rad_l, rad_r, d_center = calc_curv_rad_and_center_dist(exampleImg_bin, left_fit, right_fit, left_lane_inds, right_lane_inds)


    exampleImg_out1 = draw_lane(exampleImg, exampleImg_bin, left_fit, right_fit, Minv)

    
    lane_left=np.array([np.array((lane_data.left.x,lane_data.left.y),dtype=object).T])
    lane_right=np.array([np.array((lane_data.right.x,lane_data.right.y),dtype=object).T])

    cv2.polylines(exampleImg_out1, np.int32([lane_left]), isClosed=False, color=(0,255,0), thickness=4)
    cv2.polylines(exampleImg_out1, np.int32([lane_right]), isClosed=False, color=(0,255,0), thickness=4)
    

# edit for Using ROS

    pub()
    ic.image_send(exampleImg_out1)

    # Time elapsed
    seconds = end - start

    fps = 1 / seconds
    
    #print("Estimated frames per second : {0}".format(fps))
    #cv2.imshow('unwarp', exampleImg_unwarp)
    #cv2.imshow('canny', exampleImg_bin)
    cv2.imshow('result', result)
    #cv2.imshow('binary', exampleImg)
    cv2.imshow('final', exampleImg_out1)
    '''
    plt.imshow(exampleImg)
    x = [src[0][0],src[2][0],src[3][0],src[1][0],src[0][0]]
    y = [src[0][1],src[2][1],src[3][1],src[1][1],src[0][1]]
    plt.plot(x, y, color='#33cc99', alpha=0.4, linewidth=3, solid_capstyle='round', zorder=2)
    plt.show()
    '''
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

