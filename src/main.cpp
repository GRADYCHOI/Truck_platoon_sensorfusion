#include <ros/ros.h>
#include <iostream>
#include <array>

#include <platooning_perception/can.h>
#include <platooning_perception/multican.h>
#include "platooning_perception/kalman.hpp"
#include "platooning_perception/AdaptiveGating.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adaptive_gating");
	ros::NodeHandle nh("~");
	adaptive_gating::AdaptiveGating AG(nh);

	ros::spin();

    return 0;
}
