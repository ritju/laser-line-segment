#include <rclcpp/rclcpp.hpp>
#include "laserline/line_path_compare.hpp"
#include <unistd.h> //sleep(seconds)
#include <thread>

float poses[][2] = {
        {4.0, -5.0},
        {2.2, -5.0},
        {2.2, -4.0},
        {2.2, -3.0},
        {2.2, -2.0},
        {4.0, -2.0},
};

#define array_size 6

void test(std::shared_ptr<line_path_compare::LinePathCompare>nh, nav_msgs::msg::Path path)
{
        RCLCPP_INFO(nh->get_logger(), "***** start test thread. *****");
        sleep(5);
        auto result = nh->get_compare_result(path);
        if (result.size() > 0)
        {
                RCLCPP_INFO(nh->get_logger(), "===== results lists =====");
                for (size_t i = 0; i < result.size(); i++)
                {
                        auto path = result[i];
                        RCLCPP_INFO(nh->get_logger(), "frame_id: %s", path.header.frame_id.c_str());
                        RCLCPP_INFO(nh->get_logger(), "sec: %d, nanosec: %d", path.header.stamp.sec, path.header.stamp.nanosec);
                        RCLCPP_INFO(nh->get_logger(), "x1: %f, y1: %f", path.poses[0].pose.position.x, path.poses[0].pose.position.y);
                        RCLCPP_INFO(nh->get_logger(), "x2: %f, y2: %f", path.poses[1].pose.position.x, path.poses[1].pose.position.y);
                        RCLCPP_INFO(nh->get_logger(), "-------------------------");
                }
        }
        else
        {
                RCLCPP_WARN(nh->get_logger(), "results is empty.");
        }
}

int main(int argc,char** argv)
{       
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        for (int i = 0; i < array_size; i++)
        {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = poses[i][0];
                pose.pose.position.y = poses[i][1];
                path.poses.push_back(pose);
        }

	rclcpp::init(argc, argv);
        rclcpp::ExecutorOptions options;
        rclcpp::executors::MultiThreadedExecutor executor;
        auto nh = std::make_shared<line_path_compare::LinePathCompare>();
        auto t1 = std::thread(test, nh, path);
        t1.detach();
        
        executor.add_node(nh);
        executor.spin();
        printf("1");
        rclcpp::shutdown();      

	return 0;
}
