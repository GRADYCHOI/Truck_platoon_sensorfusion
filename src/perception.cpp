#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <platooning_perception/BoundingBoxes.h>
#include <platooning_perception/lane.h>
#include <platooning_perception/Range.h>
#include <platooning_perception/BoundingBox.h>
#include <platooning_perception/can.h>
#include <platooning_perception/multican.h>
#include <math.h>
#include <boost/thread.hpp>
#include <thread>

#include <sys/time.h>

// Socket CAN
#include <stdio.h> 
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sstream>
#include <string>

#define RADAR_msg 16

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

float pos_x[16]={0,},pos_y[16]={0,},angle[16]={0,};
int center_x[20],center_y[20],center_angle[20];
int tracking = 1,pre_index = 0;
int tracking1 = 1;
int object_num,isFirst=1;
int pre_object_num;
int tracking_count = 0;
//int rcv_num=0;

struct can_frame frame;
struct sockaddr_can addr;

Scalar red(0,0,255);
Scalar green(0,255,0);
Scalar white(255,255,255);
Scalar yellow(0,255,255);
Scalar blue(255,0,0);

platooning_perception::BoundingBoxes pre_boundingBoxes;
platooning_perception::BoundingBoxes boundingBoxes;
platooning_perception::multican can;
platooning_perception::lane lane_data; 

struct can{
    int flag[16];
    int id[16];
    int pos_x[16];
    int pos_y[16];
    int angle[16];
    int matching_index[16];
} can_data;

typedef struct value{
    float iou;
    float scale;
}val;

cv_bridge::CvImagePtr cv_ptr;

std_msgs::Header last;

class Perception 
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber bbox_sub_;
    ros::Subscriber lane_sub_;
    ros::Subscriber object_sub_;
//    ros::Subscriber can_sub_;

    public:
    Perception()
        : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
        //image_sub_ = it_.subscribe("/darknet_ros/detection_image", 1,
        //image_sub_ = it_.subscribe("/lane_image", 1,
//        image_sub_ = it_.subscribe("/videofile/image_raw", 1,
                &Perception::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        bbox_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1,
                &Perception::bboxCb, this);
        object_sub_ = nh_.subscribe("/darknet_ros/found_object", 1,
                &Perception::objectCb, this);
        lane_sub_ = nh_.subscribe("/lane_range", 1,
                &Perception::laneCb, this);
 
        //can_sub_ = nh_.subscribe("/can_msgs",1,&Perception::canCb, this);
            
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~Perception()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

   // Lane data callback 
    void laneCb(const platooning_perception::lane& msg)
    {
        printf("laneCb : %f\n", Perception::gettimeafterboot());
        lane_data = msg;

        Point lane_right_bottom(lane_data.right.x[0],lane_data.right.y[0]);
        Point lane_left_bottom(lane_data.left.x[719],lane_data.left.y[719]);
        Point lane_right_top(lane_data.right.x[719],lane_data.right.y[719]);
        Point lane_left_top(lane_data.left.x[0],lane_data.left.y[0]);
 
        line(cv_ptr->image, lane_right_bottom, lane_right_top, red, 5);
        line(cv_ptr->image, lane_left_bottom, lane_left_top, red, 5);
    }

    // Number of object callback 
    void objectCb(const std_msgs::Int8& msg)
    {
        printf("objectCb : %f\n", Perception::gettimeafterboot());
        object_num = msg.data;
    }

    // CAN data callback 
    void canCb(const platooning_perception::multican& msg)
    {
        printf("==========================\n");
        can = msg;
        for(int i=0; i<msg.num; i++){
            printf("timestamp : (%3d.%09d)\n ",msg.header.stamp.sec,msg.header.stamp.nsec);
            printf("msg ID : %3X \n",msg.can_msgs[i].id);
            int index = msg.can_msgs[i].id - 257;
            can_data.flag[index] = 1;
            can_data.matching_index[index] = 0;

            can_data.id[index] = msg.can_msgs[i].id;
            can_data.pos_x[index] = (((msg.can_msgs[i].data[2] & 0x7F)<<8) + msg.can_msgs[i].data[1])*0.01;
            can_data.pos_y[index] = (((msg.can_msgs[i].data[4] & 0x3F)<<8) + msg.can_msgs[i].data[3])*0.01-81.91;
            can_data.angle[index] = atan2(can_data.pos_y[index],can_data.pos_x[index])*57.3;

            printf("pos_x: %d\n",can_data.pos_x[index]);
            printf("pos_y: %d\n",can_data.pos_y[index]);
            printf("distance : %f\n",Perception::getDistance(can_data.pos_x[index],can_data.pos_y[index],0,0));
        }
        printf("==========================\n");
        printf("can callback\n");
    }

    // Bounding box callback 
    void bboxCb(const platooning_perception::BoundingBoxes& msg)
    {
        Scalar blue(255,0,0);
        Scalar red(255,255,0);

        boundingBoxes = msg;
        printf("bboxCb : %f\n", Perception::gettimeafterboot());

        for(int i=0; i < object_num; i++)
        {
            boundingBoxes.bounding_boxes[i].tracking_index = i;
            center_x[i] = ((msg.bounding_boxes[i].xmax-msg.bounding_boxes[i].xmin)/2.0+msg.bounding_boxes[i].xmin)*2;
            //center_x[i] = ((msg.bounding_boxes[i].xmax-msg.bounding_boxes[i].xmin)/2.0+msg.bounding_boxes[i].xmin);
            center_y[i] = ((msg.bounding_boxes[i].ymax-msg.bounding_boxes[i].ymin)/2.0+msg.bounding_boxes[i].ymin)*1.5;
            //center_y[i] = ((msg.bounding_boxes[i].ymax-msg.bounding_boxes[i].ymin)/2.0+msg.bounding_boxes[i].ymin);
            center_angle[i] = atan2(center_x[i]-640,720-center_y[i])*57.3;
        }
    }
        
    // Image callback 
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            printf("imageCb : %f\n", Perception::gettimeafterboot());
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::resize(cv_ptr->image,cv_ptr->image, Size(1280,720),CV_INTER_LINEAR);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
       image_pub_.publish(cv_ptr->toImageMsg());
    }

    // Calculate IOU & scale between before bounding box and subscribe bounding box 
    val calculateAreaProperty(int x1,int y1,int x2,int y2,int a1,int b1,int a2,int b2)
    {
        int xx1, xx2, yy1, yy2, w, h;
        float area_x,area_a;
        float iou,wh;
        val calculate;

        area_x = (x2-x1)*(y2-y1);
        area_a = (a2-a1)*(b2-b1);
        
        xx1 = min(x1, a1);
        xx2 = max(x2, a2);
        yy1 = min(y1, b1);
        yy2 = max(y2, b2);

        w = max(0, xx1-xx2);
        h = max(0, yy1-yy2);

        wh = w*h;
        calculate.iou = wh/((x1-x2) * (y1-y2) + (a1-a2) * (b1-b2) - wh);
        
        if(area_x >= area_a) calculate.scale = area_a/area_x;
        else calculate.scale = area_x/area_a;

        return calculate;
    }

    // Calculate angle difference
    int getAngleDiff(int bbox_angle, int can_angle)
    {
        return abs((float)bbox_angle-(can_angle));
    }

    // Calculate distance
    float getDistance(float x, float y,float x2, float y2)
    {
         return sqrt((x-x2)*(x-x2) + (y-y2)*(y-y2));
    }

    // Calculate arctangent
    float getAtan(float x1, float y1,float x2, float y2)
    {
         return atan2((y1-y2),(x2-x1))*57.3;
    }

    // Check tracking condition
    int checkTrackingCondition(val *area,int size)
    {
        int index = 0;

        for(int i = 0; i < size; i++){
            if((((area + index)->iou) <= ((area + i)->iou)) && ((area + i)->iou >= 0.4) && ((area+i)->scale >= 0.8)) {
                index = i;
                tracking_count++;
            }
        }
        return index;
    }

    // Get Max or Min index
    int getIndex(float *array,int size,int action,int threshold)
    {
        int index = 0;

        switch (action)
        {
            // 1 : Max Index
            case 1:

                for(int i = 0; i < size;i++){
                    if(((*(array + index)) < (*(array + i))) && (*(array + i) >= threshold)) index = i;
                }
                return index;
                break;
            // 2 : Min Index
            case 2:
        
                //printf("size : %d\n",size);
                for(int i = 0; i < size;i++){
                    if(((*(array + index)) > (*(array + i))) && (*(array + i) >= threshold)) index = i;
                }
                //printf("Index : %d\n",index);
                return index;
                break;

            default:
                return index;
        }
    }

    // Select target vehicle
    void targetSelection()
    {
        printf("==========================\n");
        printf("target selection start\n");

        float slope_left[object_num+1] = {0,};
        float slope_right[object_num+1] = {0,};
        float distance[object_num] = {0,};
        float lane_center[2],Index_list[object_num];
        int min_distance_index = 0;
        int pre_center_x,pre_center_y;
        int distance_size;

        lane_center[0] = lane_data.right.x[0] - lane_data.left.x[719];
        lane_center[1] = lane_data.right.y[0] - lane_data.left.y[719];

        slope_left[0] = Perception::getAtan(lane_data.left.x[719],lane_data.left.y[719],lane_data.left.x[0],lane_data.left.y[0]);
        slope_right[0] = Perception::getAtan(lane_data.right.x[0],lane_data.right.y[0],lane_data.right.x[719],lane_data.right.y[719]);

        //printf("reference slope : %f\n",slope_left[0]);

        for(int i = 1; i < object_num+1; i++){
            slope_left[i] = Perception::getAtan(lane_data.left.x[719],lane_data.left.y[719],center_x[i-1],center_y[i-1]);
            slope_right[i] = Perception::getAtan(lane_data.right.x[0],lane_data.right.y[0],center_x[i-1],center_y[i-1]);
            distance[i-1] = Perception::getDistance(lane_center[0],lane_center[1],center_x[i-1],center_y[i-1]); 

            //if((slope_left[0] <= 0) || (slope_left[0] >= slope_left[i]) || (slope_right[0] <= slope_right[i])) distance[i-1] = 1000;
            if((slope_left[i] > slope_left[0]) || (slope_right[i] < slope_right[0])) distance[i-1] = 1000;
         //   printf("distance%d : %f\n",i-1,distance[i-1]);
        }

        // get index of minimum value 
        distance_size = sizeof(distance)/sizeof(distance[0]);
        min_distance_index = Perception::getIndex(distance,distance_size,2,0);

        //printf("min_distance_index : %d\n",min_distance_index);
        //printf("slope : %f\n",slope_left[min_distance_index+1]);

        if(distance[min_distance_index] != 1000) {
            Point center(center_x[min_distance_index],center_y[min_distance_index]);
            pre_center_x = center_x[min_distance_index];
            pre_center_y = center_y[min_distance_index];

            circle(cv_ptr->image,center,3,blue,5);
        }
        else {
            Point center(pre_center_x,pre_center_y);
            circle(cv_ptr->image,center,3,blue,5);
        }

        printf("target selection end\n");
        printf("==========================\n");
    }

    // Object detection tracking
    void objectTracking()
    {
        printf("==========================\n");
        printf("tracking start\n");
        platooning_perception::BoundingBoxes object;
        object = boundingBoxes;

        float threshold = 0.4;
        int tracking_index = 0;

        if(isFirst) {
            pre_boundingBoxes = object;
            pre_object_num = object_num;
        }

         // printf("pre_object_num : %d\n", pre_object_num);
        for(int i=0; i < pre_object_num; i++)
        {
            float max_area = 0;
            int max_area_index = 0;
            tracking_count = 0;
            val area[object_num];

            for(int j=0; j < object_num; j++){

                area[j] = Perception::calculateAreaProperty(object.bounding_boxes[j].xmax*2 , object.bounding_boxes[j].ymax*1.5 , object.bounding_boxes[j].xmin*2 , object.bounding_boxes[j].ymin*1.5 , pre_boundingBoxes.bounding_boxes[i].xmax*2 , pre_boundingBoxes.bounding_boxes[i].ymax*1.5 , pre_boundingBoxes.bounding_boxes[i].xmin*2 , pre_boundingBoxes.bounding_boxes[i].ymin*1.5);
                //area = Perception::calculateIOU(object.bounding_boxes[j].xmax , object.bounding_boxes[j].ymax , object.bounding_boxes[j].xmin , object.bounding_boxes[j].ymin , pre_boundingBoxes.bounding_boxes[i].xmax , pre_boundingBoxes.bounding_boxes[i].ymax , pre_boundingBoxes.bounding_boxes[i].xmin , pre_boundingBoxes.bounding_boxes[i].ymin);

        //        printf("area %d : %f\n",j,area);

            }

            //printf("area : %d\n",sizeof(area));
            //printf("area1 : %d\n",sizeof(val));
 
            max_area_index = Perception::checkTrackingCondition(area,sizeof(area)/sizeof(val));

            //printf("max area index : %d\n",max_area_index);
            //printf("tracking_count : %d\n",tracking_count);

            if((tracking_count != 0) && ((object.bounding_boxes[max_area_index].Class == "train") || (object.bounding_boxes[max_area_index].Class == "car"))){

                char id[20] = "";
                char lable[10] = "ID : ";
                char temp[3];

                for(int k=0;k < object_num; k++){
          //      printf("%d index before ID : %d\n",k,object.bounding_boxes[k].tracking_index);
                    if(object.bounding_boxes[k].tracking_index == pre_boundingBoxes.bounding_boxes[i].tracking_index){
                        object.bounding_boxes[k].tracking_index = object_num + k;
                    }
                }
                object.bounding_boxes[max_area_index].tracking_index = pre_boundingBoxes.bounding_boxes[i].tracking_index;

            //    printf("%d tracking ID : %d\n",max_area_index,object.bounding_boxes[max_area_index].tracking_index);

                sprintf(temp, "%d",pre_boundingBoxes.bounding_boxes[i].tracking_index);
                tracking_index++;

                strcat(id,lable);
                strcat(id,temp);

                Point center(center_x[max_area_index],center_y[max_area_index]);
                Point text_pos(center_x[max_area_index]+20,center_y[max_area_index]);
                cv::putText(cv_ptr->image, id, text_pos, CV_FONT_ITALIC, 0.7 , green,3); 

                //matchingFunc(center_x[max_area_index],center_y[max_area_index],center_angle[max_area_index],can_data.angle,can_data.flag,can_data.matching_index); 
            }
        }
        isFirst=0;
        pre_boundingBoxes = object;
        pre_object_num = object_num;

        for(int i=0; i<RADAR_msg; i++){
            can_data.matching_index[i] = 0;
        }
        printf("Tracking end\n");

        printf("==========================\n");
    }

    // Matching with CAN data and dynamic Object  
    void matchingFunc(int pos_x,int pos_y,int angle,int *can_angle,int *can_flag,int *can_matching_index)
    {
        float angle_diff[RADAR_msg];
        int min_angle_index = 0;
        int matching_count = 0;
        int angle_threshold = 7;
        int angle_diff_size;

        for(int i=0; i<RADAR_msg; i++){

            if((*(can_flag+i) == 1) && (*(can_matching_index + i) == 0)){
                angle_diff[i] = Perception::getAngleDiff(angle,*(can_angle+i));
            }
        }

        angle_diff_size = sizeof(angle_diff)/sizeof(angle_diff[0]);
        min_angle_index = Perception::getIndex(angle_diff,angle_diff_size,2,angle_threshold);

        if(angle_diff[min_angle_index] < angle_threshold){

            char dist[20] = "";
            char lable[10] = "dist(m): ";
            //char lable[10] = "index : ";
            char temp[10] = "";

            for(int ndx = 0; ndx<frame.can_dlc; ndx++){
                printf(" %02X",can.can_msgs[min_angle_index].data[ndx]);
            }
            printf("\n");

            printf("min_angle_index : %d\n",min_angle_index);
            sprintf(temp, "%.2f", Perception::getDistance(can_data.pos_x[min_angle_index],can_data.pos_y[min_angle_index],0,0));
            //sprintf(temp, "%d",min_label+1);
            //sprintf(temp, "%d",can_data.pos_x[min_label]);
            strcat(dist,lable);
            strcat(dist,temp);

            Point center(pos_x,pos_y);
            Point text_pos(pos_x+20,pos_y+20);
            circle(cv_ptr->image,center,3,blue,5);
            cv::putText(cv_ptr->image, dist, text_pos, CV_FONT_ITALIC, 0.7 , green,2); 

            *(can_matching_index + min_angle_index) = 1;
        }
    } //matching_func 

    void trackingThread()
    {
        //        printf("==========================\n");

        double start,end;

        start = Perception::gettimeafterboot();

        platooning_perception::BoundingBoxes object;
        object = boundingBoxes;

        float threshold = 0.4;
        int tracking_index = 0;

        if(isFirst) {
            pre_boundingBoxes = object;
            pre_object_num = object_num;
        }

        // printf("pre_object_num : %d\n", pre_object_num);
        for(int i=0; i < pre_object_num; i++)
        {
            float max_area = 0;
            int max_area_index = 0;
            tracking_count = 0;
            val area[object_num];

            for(int j=0; j < object_num; j++){

                area[j] = Perception::calculateAreaProperty(object.bounding_boxes[j].xmax*2 , object.bounding_boxes[j].ymax*1.5 , object.bounding_boxes[j].xmin*2 , object.bounding_boxes[j].ymin*1.5 , pre_boundingBoxes.bounding_boxes[i].xmax*2 , pre_boundingBoxes.bounding_boxes[i].ymax*1.5 , pre_boundingBoxes.bounding_boxes[i].xmin*2 , pre_boundingBoxes.bounding_boxes[i].ymin*1.5);
                //area = Perception::calculateIOU(object.bounding_boxes[j].xmax , object.bounding_boxes[j].ymax , object.bounding_boxes[j].xmin , object.bounding_boxes[j].ymin , pre_boundingBoxes.bounding_boxes[i].xmax , pre_boundingBoxes.bounding_boxes[i].ymax , pre_boundingBoxes.bounding_boxes[i].xmin , pre_boundingBoxes.bounding_boxes[i].ymin);

                //        printf("area %d : %f\n",j,area);

            }

            //printf("area : %d\n",sizeof(area));
            //printf("area1 : %d\n",sizeof(val));

            max_area_index = Perception::checkTrackingCondition(area,sizeof(area)/sizeof(val));

            //printf("max area index : %d\n",max_area_index);
            //printf("tracking_count : %d\n",tracking_count);

            if((tracking_count != 0) && ((object.bounding_boxes[max_area_index].Class == "train") || (object.bounding_boxes[max_area_index].Class == "car"))){

                char id[20] = "";
                char lable[10] = "ID : ";
                char temp[3];

                for(int k=0;k < object_num; k++){
                    //      printf("%d index before ID : %d\n",k,object.bounding_boxes[k].tracking_index);
                    if(object.bounding_boxes[k].tracking_index == pre_boundingBoxes.bounding_boxes[i].tracking_index){
                        object.bounding_boxes[k].tracking_index = object_num + k;
                    }
                }
                object.bounding_boxes[max_area_index].tracking_index = pre_boundingBoxes.bounding_boxes[i].tracking_index;

                //    printf("%d tracking ID : %d\n",max_area_index,object.bounding_boxes[max_area_index].tracking_index);

                sprintf(temp, "%d",pre_boundingBoxes.bounding_boxes[i].tracking_index);
                tracking_index++;

                strcat(id,lable);
                strcat(id,temp);

                Point center(center_x[max_area_index],center_y[max_area_index]);
                Point text_pos(center_x[max_area_index]+20,center_y[max_area_index]);
                cv::putText(cv_ptr->image, id, text_pos, CV_FONT_ITALIC, 0.7 , green,3); 

                //matchingFunc(center_x[max_area_index],center_y[max_area_index],center_angle[max_area_index],can_data.angle,can_data.flag,can_data.matching_index); 
            }
        }
        isFirst=0;
        pre_boundingBoxes = object;
        pre_object_num = object_num;

        for(int i=0; i<RADAR_msg; i++){
            can_data.matching_index[i] = 0;
        }

        //        printf("==========================\n");

        end = Perception::gettimeafterboot();
    //    printf("tracking_time : %f\n", end-start);
    }

    void targetingThread()
    {
    //    double s,e;

     //   s = perception::gettimeafterboot();

        float slope_left[object_num+1] = {0,};
        float slope_right[object_num+1] = {0,};
        float distance[object_num] = {0,};
        float lane_center[2],Index_list[object_num];
        int min_distance_index = 0;
        int pre_center_x,pre_center_y;
        int distance_size;

        lane_center[0] = lane_data.right.x[0] - lane_data.left.x[719];
        lane_center[1] = lane_data.right.y[0] - lane_data.left.y[719];

        slope_left[0] = Perception::getAtan(lane_data.left.x[719],lane_data.left.y[719],lane_data.left.x[0],lane_data.left.y[0]);
        slope_right[0] = Perception::getAtan(lane_data.right.x[0],lane_data.right.y[0],lane_data.right.x[719],lane_data.right.y[719]);

        //printf("reference slope : %f\n",slope_left[0]);

        for(int i = 1; i < object_num+1; i++){
            slope_left[i] = Perception::getAtan(lane_data.left.x[719],lane_data.left.y[719],center_x[i-1],center_y[i-1]);
            slope_right[i] = Perception::getAtan(lane_data.right.x[0],lane_data.right.y[0],center_x[i-1],center_y[i-1]);
            distance[i-1] = Perception::getDistance(lane_center[0],lane_center[1],center_x[i-1],center_y[i-1]); 

            //if((slope_left[0] <= 0) || (slope_left[0] >= slope_left[i]) || (slope_right[0] <= slope_right[i])) distance[i-1] = 1000;
            if((slope_left[i] > slope_left[0]) || (slope_right[i] < slope_right[0])) distance[i-1] = 1000;
         //   printf("distance%d : %f\n",i-1,distance[i-1]);
        }

        // get index of minimum value 
        distance_size = sizeof(distance)/sizeof(distance[0]);
        min_distance_index = Perception::getIndex(distance,distance_size,2,0);

        //printf("min_distance_index : %d\n",min_distance_index);
        //printf("slope : %f\n",slope_left[min_distance_index+1]);

        if(distance[min_distance_index] != 1000) {
            Point center(center_x[min_distance_index],center_y[min_distance_index]);
            pre_center_x = center_x[min_distance_index];
            pre_center_y = center_y[min_distance_index];

            circle(cv_ptr->image,center,3,blue,5);
        }
        else {
            Point center(pre_center_x,pre_center_y);
            circle(cv_ptr->image,center,3,blue,5);
        }

        
//        e = Perception::gettimeafterboot();
//        printf("targetting_time : %f\n", e-s);
    }

    double gettimeafterboot()
    {
        struct timespec time_after_boot;
        clock_gettime(CLOCK_MONOTONIC,&time_after_boot);
        return (time_after_boot.tv_sec*1000+time_after_boot.tv_nsec*0.000001);
    }

}; //Perception class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking");
    //ros::init(argc, argv, "platooning_perception");

    Perception perception;
    ros::Rate loop_rate(100);

    std::thread tracking_thread;
    std::thread targeting_thread;

    int count = 0;
   
    double time[100];

    while(ros::ok())
    {
        double s,e;
        double sum=0;

        s = perception.gettimeafterboot();

        ros::spinOnce();
        //printf("==========================\n");

        //tracking_thread = std::thread(&Perception::trackingThread, &perception);
        //targeting_thread = std::thread(&Perception::targetingThread, &perception);

        perception.targetSelection();
        perception.objectTracking();

        //tracking_thread.join();
        //targeting_thread.join();

        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        loop_rate.sleep();

        e = perception.gettimeafterboot();

        if(count < 100){
            time[count] = e-s;
            //printf("targetting_time%d : %f\n",count ,time[count]);
            count++;
        }
        else {
            for(int i=0; i < 100; i++){
                sum += time[i];
            }
            //printf("average_time : %f\n", sum/100.0);
        }

        usleep(3000);
    //    printf("==========================\n");
    }

    return 0;
}
