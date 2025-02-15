/*
mobile robot pose from laser
*/

#ifndef _LASER_FEATURE_ROS_H_
#define _LASER_FEATURE_ROS_H_

#include <vector>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "laserline/line_feature.h"

namespace line_feature
{

class LaserFeatureROS
{
    public:
		LaserFeatureROS(rclcpp::Node::SharedPtr NodeHandle, rclcpp::Node::SharedPtr);
		//
		~LaserFeatureROS();
		//
		void startgame();
	private:
		//memeber function
		//发布直线分割消息
		void publishMarkerMsg(const std::vector<gline> &,visualization_msgs::msg::Marker &marker_msg);
		//开始函数，包括读取文件（先验地图等信息）
		//load params
		void load_params();
		//角度参量
		void compute_bearing(const sensor_msgs::msg::LaserScan::ConstPtr&);
		//激光线程函数（ros节点回调函数），采集激光并进行处理，处理频率以采集频率为准
		void scanValues(const sensor_msgs::msg::LaserScan::ConstPtr&);

     private:
		//参数信息laser
		bool com_bearing_flag;
		bool show_lines_;
		double m_startAng;
		double m_AngInc;
		LineFeature line_feature_;

		std::string frame_id_;
  		std::string scan_topic_;

		std::vector<gline> m_gline;
		std::vector<line> m_line;
		//ROS
		rclcpp::Node::SharedPtr nh_;
  		rclcpp::Node::SharedPtr nh_local_;
  		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  		// rclcpp::Publisher<>::SharedPtr line_publisher_;
  		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

}
#endif

