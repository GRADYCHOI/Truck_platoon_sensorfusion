#include <array>
#include <algorithm>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <platooning_perception/can.h>
#include <platooning_perception/multican.h>


using namespace std;

namespace adaptive_gating {

class AdaptiveGating
{
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;

	visualization_msgs::Marker mk_camera;
	visualization_msgs::Marker mk_radar;
	visualization_msgs::Marker mk_fusion;
	visualization_msgs::Marker mk_ffusion;
	visualization_msgs::Marker mk_origin;
	visualization_msgs::Marker mk_L_lane;
	visualization_msgs::Marker mk_R_lane;
	visualization_msgs::Marker mk_LL_lane;
	visualization_msgs::Marker mk_RR_lane;
	visualization_msgs::Marker mk_esr;
	visualization_msgs::MarkerArray ma;
	
	vector<float> id_both_n;
	vector<float> id_both_m;
	vector<float> id_camera_only;
	vector<float> id_radar_only;
	vector<float> cx_float;
	vector<float> cy_float;
	vector<float> rx_float;
	vector<float> ry_float;
	vector<float> fx_float;
	vector<float> fy_float;
	vector<float> ffx_float;
	vector<float> ffy_float;
	vector<float> esrx_float;
	vector<float> esry_float;

	vector<int> vec[10];

	vector<float> camera_gate_xx;
	vector<float> camera_gate_yy;
	vector<float> radar_gate_xx;
	vector<float> radar_gate_yy;

	bool filter_switch[16] = {false};
	vector<float> r_switch;

	int threshold_x = 60;
	int threshold_y = 80;
	
	float radar_pos_x = 0, radar_pos_y = 0;
	float camera_pos_x = 0, camera_pos_y = 0;
	float fusion_pos_x = 0, fusion_pos_y = 0;
	float ffusion_pos_x = 0, ffusion_pos_y = 0;
	float esr_pos_x = 0, esr_pos_y = 0;

	float L_lane_curvature_derivative;
	float L_lane_curvature;
	float L_lane_position;
	float L_heading_angle;

	float R_lane_curvature_derivative;
	float R_lane_curvature;
	float R_lane_position;
	float R_heading_angle;
	
//	array<float, 16> radar_x = {0};
//	array<float, 16> radar_y = {0};
//	array<float, 10> camera_x = {0};
//	array<float, 10> camera_y = {0};
//	array<float, 10> camera_w = {0};
	array<float, 64> radar_x = {0};
	array<float, 64> radar_y = {0};
	array<float, 30> camera_x = {0};
	array<float, 30> camera_y = {0};
	array<float, 30> camera_w = {0};
	array<float, 16> fusion_x = {0};
	array<float, 16> fusion_y = {0};
	array<float, 16> ffusion_x = {0};
	array<float, 16> ffusion_y = {0};
	array<float, 16> esr_x = {0};
	array<float, 16> esr_y = {0};

	array<float, 16> filteredValue_x = {0};
	array<float, 16> filteredValue_y = {0};
	
	void getXY(const platooning_perception::multican& msg);
	void getGateSize();
	void getMarker();
	void getLine();
	void initMarker();
	void clearVec();
	void Fusion();
	void init_LowPassFilter();
	void LowPassFilter();
	void getTarget();
	void Callback(const platooning_perception::multican& msg);


public:

	explicit AdaptiveGating(ros::NodeHandle node);
	~AdaptiveGating();
};

}
