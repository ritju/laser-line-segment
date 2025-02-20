#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "wall_line_detection_msgs/msg/wall_lines_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <mutex>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

//点信息
typedef struct _POINT
{
	double x;
	double y;
}POINT;

namespace line_path_compare
{        
        class LinePathCompare : public rclcpp::Node
        {
        public:
                
                explicit LinePathCompare(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
                ~LinePathCompare();

                std::vector<nav_msgs::msg::Path> get_compare_result();

                // subs
                rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
                rclcpp::Subscription<wall_line_detection_msgs::msg::WallLinesStamped>::SharedPtr wall_lines_sub_;

                // callback for subs
                void path_sub_callback_(nav_msgs::msg::Path::SharedPtr msg);
                void wall_lines_callback_(wall_line_detection_msgs::msg::WallLinesStamped::SharedPtr msg);

                bool is_current(double);
                bool is_similar(POINT line1_p1, POINT line1_p2, POINT line2_p1, POINT line2_p2, double theta_thr, double dis_thr);

                double area(int x1, int y1, int x2, int y2, int x3, int y3);
                double distance(int x1, int y1, int x2, int y2);
                double calculate_height(int x1, int y1, int x2, int y2, int x3, int y3);

                void get_tf(std::string laser_frame_id, rclcpp::Time laser_scan_time);

                nav_msgs::msg::Path path_;
                wall_line_detection_msgs::msg::WallLinesStamped wall_lines_;
                std::vector<nav_msgs::msg::Path> result_;
                bool path_is_current_{false}, wall_lines_is_current{false};
                double path_last_received_time;
                double wall_lines_last_received_time;
                double result_time;

                std::mutex mutex;

                // tf2
                std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
                std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
                tf2::Transform map_laser_link_tf;

                // parameters
                double theta_thr_;
                double dis_thr_;
                std::string path_topic_;
                std::string wall_lines_topic_;
                double time_tolerance_;
                double tf_tolerance_;

                void init_params();

        };  // end of class


} // end of namespace



