#include "laserline/line_path_compare.hpp"

namespace line_path_compare
{
        LinePathCompare::LinePathCompare(const rclcpp::NodeOptions& options): rclcpp::Node("line_path_compare_node", options)
        {
                RCLCPP_INFO(get_logger(), "line_path_compare_node construction");
                init_params();

                // init tf2
                this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

                auto callback_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops1 = rclcpp::SubscriptionOptions();
                sub_ops1.callback_group = callback_group1;
                path_sub_ = this->create_subscription<nav_msgs::msg::Path>(path_topic_, 1, 
                        std::bind(&LinePathCompare::path_sub_callback_, this, std::placeholders::_1), sub_ops1);

                auto callback_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops2 = rclcpp::SubscriptionOptions();
                sub_ops2.callback_group = callback_group2;
                wall_lines_sub_ = this->create_subscription<wall_line_detection_msgs::msg::WallLinesStamped>(wall_lines_topic_, 1,
                        std::bind(&LinePathCompare::wall_lines_callback_, this, std::placeholders::_1), sub_ops2);

        }

        LinePathCompare::~LinePathCompare()
        {
              RCLCPP_INFO(get_logger(), "line_path_compare_node destruction");  
        }

        void LinePathCompare::init_params()
        {
                this->declare_parameter<double>("theta_thr", 0.05);
                this->declare_parameter<double>("dis_thr", 0.1);
                this->declare_parameter<std::string>("wall_lines_topic", "wall_lines_topic");
                this->declare_parameter<std::string>("path_topic", "path_topic");
                this->declare_parameter<double>("time_tolerance", 0.1);
                this->declare_parameter<double>("tf_tolerance", 0.1);

                this->theta_thr_ = this->get_parameter_or<double>("theta_thr", 0.05);
                this->dis_thr_ = this->get_parameter_or<double>("dis_thr", 0.1);
                this->wall_lines_topic_ = this->get_parameter_or<std::string>("wall_lines_topic", "wall_lines_topic");
                this->path_topic_ = this->get_parameter_or<std::string>("path_topic", "path_topic");
                this->time_tolerance_ = this->get_parameter_or<double>("time_tolerance", 0.1);
                this->tf_tolerance_ = this->get_parameter_or<double>("tf_tolerance", 0.1);

                RCLCPP_INFO(get_logger(), "---------- show all the parameters ----------");
                RCLCPP_INFO(get_logger(), "theta_thr: %f", theta_thr_);
                RCLCPP_INFO(get_logger(), "dis_thr: %f", dis_thr_);
                RCLCPP_INFO(get_logger(), "wall_lines_topic: %s", wall_lines_topic_.c_str());
                RCLCPP_INFO(get_logger(), "path_topic: %s", path_topic_.c_str());
                RCLCPP_INFO(get_logger(), "time_tolerance: %f", time_tolerance_);
                RCLCPP_INFO(get_logger(), "tf_tolerance: %f", tf_tolerance_);
        }

        void LinePathCompare::path_sub_callback_(nav_msgs::msg::Path::SharedPtr msg)
        {
                mutex.lock();

                result_.clear();
                path_ = *msg;
                path_last_received_time = now().seconds();
                size_t poses_size = msg->poses.size();
                std::vector<std::pair<POINT,POINT>> path_vec, wall_line_vec;
                for (size_t i = 0; i < poses_size - 1; i++)
                {
                        std::pair<POINT,POINT> line;
                        line.first.x = path_.poses[i].pose.position.x;
                        line.first.y = path_.poses[i].pose.position.y;
                        line.second.x = path_.poses[i+1].pose.position.x;
                        line.second.y = path_.poses[i+1].pose.position.y;
                        path_vec.push_back(line);
                }

                for (size_t i = 0; i < wall_lines_.wall_lines.size(); i++)
                {
                        auto wall_line = wall_lines_.wall_lines[i];
                        std::pair<POINT,POINT> line;
                        line.first.x = wall_line.x1;
                        line.first.y = wall_line.y1;
                        line.second.x = wall_line.x2;
                        line.second.y = wall_line.y2;
                        wall_line_vec.push_back(line);
                }

                for (size_t i = 0; i < path_vec.size(); i++)
                {
                        for (size_t j = 0; j < wall_line_vec.size(); j++)
                        {
                                POINT line1_p1, line1_p2, line2_p1, line2_p2;

                                line1_p1.x = path_vec[i].first.x;
                                line1_p1.y = path_vec[i].first.y;
                                line1_p2.x = path_vec[i].second.x;
                                line1_p2.y = path_vec[i].second.y;

                                line2_p1.x = wall_line_vec[j].first.x;
                                line2_p1.y = wall_line_vec[j].first.y;
                                line2_p2.x = wall_line_vec[j].second.x;
                                line2_p2.y = wall_line_vec[j].second.y;

                                if (is_similar(line1_p1, line1_p2, line2_p1, line2_p2, theta_thr_, dis_thr_))
                                {
                                        nav_msgs::msg::Path path;
                                        path.header= msg->header;
                                        geometry_msgs::msg::PoseStamped pose;
                                        pose = msg->poses[i];
                                        path.poses.push_back(pose);
                                        pose = msg->poses[i+1];
                                        path.poses.push_back(pose);
                                        result_.push_back(path);
                                        break;
                                }
                                else
                                {
                                        continue;
                                }
                        }
                }

                mutex.unlock();
        }

        void LinePathCompare::wall_lines_callback_(wall_line_detection_msgs::msg::WallLinesStamped::SharedPtr msg)
        {
                mutex.lock();
                wall_lines_ = *msg;
                wall_lines_last_received_time = now().seconds();
                get_tf("laser_link", rclcpp::Time(msg->header.stamp));
                for(size_t i = 0; i < wall_lines_.wall_lines.size(); i++)
                {
                        tf2::Transform tf_temp, tf_;                        
                        auto wall_line = wall_lines_.wall_lines[i];

                        tf_temp.setIdentity();
                        tf_temp.setOrigin(tf2::Vector3(wall_line.x1, wall_line.y1, 0.0));
                        tf_ = map_laser_link_tf * tf_temp;
                        wall_lines_.wall_lines[i].x1 = tf_.getOrigin().getX();
                        wall_lines_.wall_lines[i].y1 = tf_.getOrigin().getY();

                        tf_temp.setIdentity();
                        tf_temp.setOrigin(tf2::Vector3(wall_line.x2, wall_line.y2, 0.0));
                        tf_ = map_laser_link_tf * tf_temp;
                        wall_lines_.wall_lines[i].x2 = tf_.getOrigin().getX();
                        wall_lines_.wall_lines[i].y2 = tf_.getOrigin().getY();

                        RCLCPP_DEBUG(get_logger(), "x1: %f, y1: %f", wall_lines_.wall_lines[i].x1, wall_lines_.wall_lines[i].y1);
                        RCLCPP_DEBUG(get_logger(), "x2: %f, y2: %f", wall_lines_.wall_lines[i].x2, wall_lines_.wall_lines[i].y2);
                }
                mutex.unlock();
        }

        std::vector<nav_msgs::msg::Path> LinePathCompare::get_compare_result()
        {
                mutex.lock();
                std::vector<nav_msgs::msg::Path> output;

                if (is_current(wall_lines_last_received_time) && is_current(path_last_received_time))
                {
                        output = result_;
                }
                mutex.unlock();

                return output;
        }

        bool LinePathCompare::is_current(double received_time)
        {
                return std::abs(now().seconds() - received_time) < this->time_tolerance_;
        }

        bool LinePathCompare::is_similar(POINT line1_p1, POINT line1_p2, POINT line2_p1, POINT line2_p2, double theta_thr,double dis_thr)
        {
                bool ret = true;

                double theta1_1, theta1_2, theta2, line_distance;
                theta1_1 = std::atan2(line1_p2.y - line1_p1.y, line1_p2.x - line1_p1.x);
                theta1_2 = std::atan2(line1_p1.y - line1_p2.y, line1_p1.x - line1_p2.x);
                theta2 = std::atan2(line2_p2.y - line2_p1.y, line2_p2.x - line2_p1.x);
                double theta_delta_1, theta_delta_2;
                theta_delta_1 = std::abs(theta1_1 - theta2);
                theta_delta_2 = std::abs(theta1_2 - theta2);
                if (theta_delta_1 >= this->theta_thr_ && theta_delta_2 >= this->theta_thr_)
                {
                        return false;
                }

                POINT line1_middle_point;
                line1_middle_point.x = (line1_p1.x + line1_p2.x) / 2.0;
                line1_middle_point.y = (line1_p1.y + line1_p2.y) / 2.0;
                line_distance = calculate_height(line1_middle_point.x, line1_middle_point.y, line2_p1.x, line2_p1.y, line2_p2.x, line2_p2.y);
                if (line_distance >= dis_thr)
                {
                        return false;
                }

                return ret;
        }

        void LinePathCompare::get_tf(std::string laser_frame, rclcpp::Time laser_scan_time)
        {
                std::string errMsg;
                std::string refFrame = "map";
                std::string childFrame = laser_frame;
                geometry_msgs::msg::TransformStamped transformStamped;

                if (!this->tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
                        tf2::durationFromSec(0.2), &errMsg))
                {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get TF from " 
                        << refFrame << " to " << childFrame << ": " << errMsg);
                } 
                else 
                {
                        try 
                        {
                                transformStamped = this->tf_buffer_->lookupTransform( refFrame, childFrame, laser_scan_time, tf2::durationFromSec(this->tf_tolerance_));
                                tf2::fromMsg(transformStamped.transform, this->map_laser_link_tf);               
                        } 
                        catch (const tf2::TransformException & e) 
                        {
                                RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
                        }
                }
        }

        double LinePathCompare::area(int x1, int y1, int x2, int y2, int x3, int y3)
        {
                return fabs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        }

        double LinePathCompare::distance(int x1, int y1, int x2, int y2)
        {     
                return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
        }

        double LinePathCompare::calculate_height(int x1, int y1, int x2, int y2, int x3, int y3)
        {
                return area(x1, y1, x2, y2, x3, y3) / distance(x2, y2, x3, y3);
        }     

} // end of namespace


RCLCPP_COMPONENTS_REGISTER_NODE(line_path_compare::LinePathCompare)


