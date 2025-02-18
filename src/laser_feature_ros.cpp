#include "laserline/laser_feature_ros.h"
#include "sys/time.h"

//FILE *fpd = fopen("data.txt","w+");
namespace line_feature
{

LaserFeatureROS::LaserFeatureROS(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr nh_local):
	nh_(nh),
	nh_local_(nh_local),
	com_bearing_flag(false)
{
	load_params();
	scan_subscriber_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS(), std::bind(&LaserFeatureROS::scanValues, this, std::placeholders::_1));
	if(show_lines_)
	{
		marker_publisher_ = nh_->create_publisher<visualization_msgs::msg::Marker>("publish_line_markers", 1);
	}
	rclcpp::spin(nh_);
}

LaserFeatureROS::~LaserFeatureROS()
{

}

void LaserFeatureROS::compute_bearing(const sensor_msgs::msg::LaserScan::ConstPtr &scan_msg)
{
	double angle_increment_,angle_start_, range_min_, range_max_;
	angle_increment_ = scan_msg->angle_increment;
	angle_start_ = scan_msg->angle_min;
	range_min_ = scan_msg->range_min;
	range_max_ = scan_msg->range_max;
	
	line_feature_.set_angle_increment(angle_increment_);
	line_feature_.set_angle_start(angle_start_);
	line_feature_.set_range_min(range_min_);
	line_feature_.set_range_max(range_max_);
	
	std::vector<double> bearings, cos_bearings, sin_bearings;
	std::vector<unsigned int> index;
	unsigned int i = 0;
	for (double b = scan_msg->angle_min; b <= scan_msg->angle_max; b += scan_msg->angle_increment)
	{
		bearings.push_back(b);
		cos_bearings.push_back(cos(b));
		sin_bearings.push_back(sin(b));
		index.push_back(i);
		i++;
	}

	line_feature_.setCosSinData(bearings, cos_bearings, sin_bearings, index);
	RCLCPP_INFO(nh_local_->get_logger(), "Data has been cached.");
}

void LaserFeatureROS::scanValues(const sensor_msgs::msg::LaserScan::ConstPtr &scan_msg)
{
	if(!com_bearing_flag)
	{
		compute_bearing(scan_msg);
		com_bearing_flag = true;
	}
	
	std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
	line_feature_.setRangeData(scan_ranges_doubles);

	startgame();
}

void LaserFeatureROS::publishMarkerMsg(const std::vector<gline> &m_gline,visualization_msgs::msg::Marker &marker_msg)
{
	marker_msg.ns = "line_extraction";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
	marker_msg.scale.x = 0.1;
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;
	marker_msg.color.a = 1.0;
  

	for (std::vector<gline>::const_iterator cit = m_gline.begin(); cit != m_gline.end(); ++cit)
	{
		geometry_msgs::msg::Point p_start;
		p_start.x = cit->x1;
		p_start.y = cit->y1;
		p_start.z = 0;
		marker_msg.points.push_back(p_start);
		geometry_msgs::msg::Point p_end;
		p_end.x = cit->x2;
		p_end.y = cit->y2;
		p_end.z = 0;
		marker_msg.points.push_back(p_end);
	}
	marker_msg.header.frame_id = frame_id_;
	marker_msg.header.stamp = nh_local_->now();
}


//主函数
void LaserFeatureROS::startgame()
{
	std::vector<line> lines;
	std::vector<gline> glines;
  	line_feature_.extractLines(lines,glines);

	// 去除以激光为起点的误检测直线。
	// if (glines.size() > 0)
	// {
	// 	for (auto it = glines.begin(); it != glines.end();)
	// 	{
	// 		double distance = std::hypot(it->x1, it->y1);
	// 		double distance_thr = 0.2;
	// 		if (distance < distance_thr)
	// 		{
	// 			RCLCPP_INFO_THROTTLE(nh_local_->get_logger(), *nh_local_->get_clock(), 1000, "x1: %f, y1: %f, distance: %f < %f, this is a bug, tmp delete", it->x1, it->y1, distance, distance_thr);
	// 			glines.erase(it);
	// 			continue;
	// 		}
	// 		it++;
	// 	}
	// }

  	// Also publish markers if parameter publish_markers is set to true
  	if (show_lines_)
  	{
  		visualization_msgs::msg::Marker marker_msg;
    		publishMarkerMsg(glines, marker_msg);
		// RCLCPP_INFO(nh_local_->get_logger(), "line size: %ld", marker_msg.points.size() / 2);
  		marker_publisher_->publish(marker_msg);
 	}
}

// Load ROS parameters
void LaserFeatureROS::load_params()
{
	RCLCPP_INFO(nh_local_->get_logger(), "*************************************");
	RCLCPP_INFO(nh_local_->get_logger(), "PARAMETERS:");

	nh_local_->declare_parameter<std::string>("frame_id", "laser_link");
	nh_local_->declare_parameter<std::string>("scan_topic", "scan");
	nh_local_->declare_parameter<bool>("show_lines", true);
	
	frame_id_ = nh_local_->get_parameter_or<std::string>("frame_id", "laser_link");
	scan_topic_ = nh_local_->get_parameter_or<std::string>("scan_topic", "scan");
	show_lines_ = nh_local_->get_parameter_or<bool>("show_lines", true);

	RCLCPP_INFO(nh_local_->get_logger(), "frame_id: %s", frame_id_.c_str());
	RCLCPP_INFO(nh_local_->get_logger(), "scan_topic: %s", scan_topic_.c_str());
	RCLCPP_INFO(nh_local_->get_logger(), "show_lines: %s", show_lines_ ? "true" : "false");

	// Parameters used by the line extraction algorithm
	int min_line_points,seed_line_points;
	double least_thresh,min_line_length,predict_distance;

	nh_local_->declare_parameter<double>("least_thresh", 0.04);
	nh_local_->declare_parameter<double>("min_line_length", 1.5);
	nh_local_->declare_parameter<double>("predict_distance", 0.1);
	nh_local_->declare_parameter<int>("seed_line_points", 6);
	nh_local_->declare_parameter<int>("min_line_points", 12);

	least_thresh = nh_local_->get_parameter_or<double>("least_thresh", 0.04);
	min_line_length = nh_local_->get_parameter_or<double>("min_line_length", 1.5);
	predict_distance = nh_local_->get_parameter_or<double>("predict_distance", 0.1);
	seed_line_points = nh_local_->get_parameter_or<int>("seed_line_points", 6);
	min_line_points = nh_local_->get_parameter_or<int>("min_line_points", 12);

	RCLCPP_INFO(nh_local_->get_logger(), "least_thresh: %lf", least_thresh);
	RCLCPP_INFO(nh_local_->get_logger(), "min_line_length: %lf", min_line_length);
	RCLCPP_INFO(nh_local_->get_logger(), "predict_distance: %lf", predict_distance);
	RCLCPP_INFO(nh_local_->get_logger(), "seed_line_points: %d", seed_line_points);
  	RCLCPP_INFO(nh_local_->get_logger(), "min_line_points: %d", min_line_points);

	line_feature_.set_least_threshold(least_thresh);
	line_feature_.set_min_line_length(min_line_length);  
	line_feature_.set_predict_distance(predict_distance);
	line_feature_.set_seed_line_points(seed_line_points);
  	line_feature_.set_min_line_points(min_line_points);

  	RCLCPP_INFO(nh_local_->get_logger(), "*************************************");
}

}
