#include "laserline/line_path_compare.hpp"

namespace line_path_compare
{
        LinePathCompare::LinePathCompare(nav2_util::LifecycleNode::SharedPtr node) : node_(node)
        {
                RCLCPP_INFO(node_->get_logger(), "line_path_compare_node construction");
                init_params();

                // init tf2
                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
                auto callback_group2 = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops2 = rclcpp::SubscriptionOptions();
                sub_ops2.callback_group = callback_group2;
                wall_lines_sub_ = node_->create_subscription<wall_line_detection_msgs::msg::WallLinesStamped>(wall_lines_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
                                  std::bind(&LinePathCompare::wall_lines_callback_, this, std::placeholders::_1), sub_ops2);

        }

        LinePathCompare::~LinePathCompare()
        {
              RCLCPP_INFO(node_->get_logger(), "line_path_compare_node destruction");  
        }

        void LinePathCompare::init_params()
        {
                node_->declare_parameter<double>("theta_thr", 0.05);
                node_->declare_parameter<double>("dis_thr", 0.1);
                node_->declare_parameter<std::string>("wall_lines_topic", "wall_lines_topic");
                node_->declare_parameter<double>("time_tolerance", 0.1);
                node_->declare_parameter<double>("tf_tolerance", 0.1);
                node_->declare_parameter<std::string>("laser_link_frame", "laser_link");

                node_->get_parameter_or<double>("theta_thr", theta_thr_, 0.5);
                node_->get_parameter_or<double>("dis_thr", dis_thr_, 1.5);
                node_->get_parameter_or<std::string>("wall_lines_topic", wall_lines_topic_, "wall_lines_topic");
                node_->get_parameter_or<double>("time_tolerance", time_tolerance_, 0.1);
                node_->get_parameter_or<double>("tf_tolerance", tf_tolerance_, 0.1);
                node_->get_parameter_or<std::string>("laser_link_frame", laser_link_frame_, "laser_link");

                RCLCPP_INFO(node_->get_logger(), "---------- show all the parameters ----------");
                RCLCPP_INFO(node_->get_logger(), "theta_thr: %f", theta_thr_);
                RCLCPP_INFO(node_->get_logger(), "dis_thr: %f", dis_thr_);
                RCLCPP_INFO(node_->get_logger(), "wall_lines_topic: %s", wall_lines_topic_.c_str());
                RCLCPP_INFO(node_->get_logger(), "time_tolerance: %f", time_tolerance_);
                RCLCPP_INFO(node_->get_logger(), "tf_tolerance: %f", tf_tolerance_);
                RCLCPP_INFO(node_->get_logger(), "laser_link_frame: %f", laser_link_frame_);
        }

        void LinePathCompare::path_process_(nav_msgs::msg::Path msg)
        {
                result_.clear();
                path_ = msg;
                path_last_received_time = node_->now().seconds();
                size_t poses_size = msg.poses.size();
                std::vector<std::pair<POINT,POINT>> path_vec, wall_line_vec;
                if  (poses_size > 2)
                {
                        std::pair<POINT,POINT> line;
                        line.first.x = path_.poses.front().pose.position.x;
                        line.first.y = path_.poses.front().pose.position.y;
                        line.second.x = path_.poses.back().pose.position.x;
                        line.second.y = path_.poses.back().pose.position.y;
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
                                        path.header.frame_id = "map";
                                        path.header.stamp = wall_lines_.laser_scan.header.stamp;
                                        geometry_msgs::msg::PoseStamped pose;
                                        pose.pose.position.x = line2_p1.x;
                                        pose.pose.position.y = line2_p1.y;
                                        path.poses.push_back(pose);
                                        pose.pose.position.x = line2_p2.x;
                                        pose.pose.position.y = line2_p2.y;
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
        }

        void LinePathCompare::wall_lines_callback_(wall_line_detection_msgs::msg::WallLinesStamped::SharedPtr msg)
        {
                mutex.lock();
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "received wall_lines_stamped msg");
                wall_lines_ = *msg;
                wall_lines_last_received_time = node_->now().seconds();
                get_tf(msg->header.frame_id, rclcpp::Time(msg->header.stamp));
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

                        RCLCPP_DEBUG(node_->get_logger(), "x1: %f, y1: %f", wall_lines_.wall_lines[i].x1, wall_lines_.wall_lines[i].y1);
                        RCLCPP_DEBUG(node_->get_logger(), "x2: %f, y2: %f", wall_lines_.wall_lines[i].x2, wall_lines_.wall_lines[i].y2);
                }
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "wall_lines_ size: %ld", wall_lines_.wall_lines.size());
                mutex.unlock();
        }

        std::vector<nav_msgs::msg::Path> LinePathCompare::get_compare_result(nav_msgs::msg::Path path)
        {
                mutex.lock();

                path_process_(path);
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
                return std::abs(node_->now().seconds() - received_time) < this->time_tolerance_;
        }

        // line1: path, line2: wall_line
        bool LinePathCompare::is_similar(POINT line1_p1, POINT line1_p2, POINT line2_p1, POINT line2_p2, double theta_thr,double dis_thr)
        {
                bool ret = true;

                // bool path_in_field = isProjectionOutside(line1_p1, line1_p2, line2_p1, line2_p2);
                // if (path_in_field)
                // {
                //         return false;
                // }

                double theta1_1, theta1_2, theta2, line_distance;
                theta1_1 = std::atan2(line1_p2.y - line1_p1.y, line1_p2.x - line1_p1.x);
                theta1_2 = std::atan2(line1_p1.y - line1_p2.y, line1_p1.x - line1_p2.x);
                theta2 = std::atan2(line2_p2.y - line2_p1.y, line2_p2.x - line2_p1.x);


                // 计算两条直线的方向向量[1](@ref)
                double v1x = line1_p2.x - line1_p1.x;
                double v1y = line1_p2.y - line1_p1.y;
                double v2x = line2_p2.x - line2_p1.x;
                double v2y = line2_p2.y - line2_p1.y;

                // 计算向量点积[6,7](@ref)
                double dot_product = v1x * v2x + v1y * v2y;

                // 计算向量模长
                double norm_v1 = std::hypot(v1x, v1y);  // 更安全的模长计算
                double norm_v2 = std::hypot(v2x, v2y);

                // 处理非法输入（重合点）
                if (norm_v1 == 0 || norm_v2 == 0) {
                        throw std::invalid_argument("直线的两个点重合，无法确定方向向量");
                }

                // 计算余弦值（取绝对值确保锐角）[2](@ref)
                double cos_theta = std::abs(dot_product) / (norm_v1 * norm_v2);

                // 处理浮点精度溢出
                cos_theta = std::max(std::min(cos_theta, 1.0), -1.0);

                double theta_delta = std::acos(cos_theta);


                RCLCPP_DEBUG(node_->get_logger(), "line2_p1: (%f, %f) , line2_p2:  (%f, %f)", line2_p1.x, line2_p1.y, line2_p2.x, line2_p2.y);
                RCLCPP_DEBUG(node_->get_logger(), "theta_delta: %f", theta_delta);
                
                if (theta_delta > this->theta_thr_)
                {
                        return false;
                }

                POINT line1_middle_point;
                line1_middle_point.x = (line1_p1.x + line1_p2.x) / 2.0;
                line1_middle_point.y = (line1_p1.y + line1_p2.y) / 2.0;
                line_distance = calculate_height(line1_middle_point.x, line1_middle_point.y, line2_p1.x, line2_p1.y, line2_p2.x, line2_p2.y);

                RCLCPP_DEBUG(node_->get_logger(), "line1_middle_point.x: %f, line1_middle_point.y: %f", line1_middle_point.x, line1_middle_point.y);
                RCLCPP_DEBUG(node_->get_logger(), "line_distance: %f", line_distance);

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
                        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to get TF from " 
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
                                node_->get_logger(),
                                "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
                        }
                }
        }

        double LinePathCompare::area(double x1, double y1, double x2, double y2, double x3, double y3)
        {
                return fabs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        }

        double LinePathCompare::distance(double x1, double y1, double x2, double y2)
        {     
                return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
        }

        double LinePathCompare::calculate_height(double x1, double y1, double x2, double y2, double x3, double y3)
        {
                return area(x1, y1, x2, y2, x3, y3) / distance(x2, y2, x3, y3);
        }
        bool LinePathCompare::isProjectionOutside(POINT line1_p1, POINT line1_p2, POINT line2_p1, POINT line2_p2) 
        {
                // 计算线段line2的向量
                const POINT line2_vec = {line2_p2.x - line2_p1.x, line2_p2.y - line2_p1.y};
                const double line2_length_sq = line2_vec.x * line2_vec.x + line2_vec.y * line2_vec.y;

                // 处理线段长度为0的特殊情况
                if (line2_length_sq < 1e-10) return true;

                // 计算line1两个端点在线段line2上的投影参数λ
                auto computeLambda = [&](POINT p) {
                const POINT vec = {p.x - line2_p1.x, p.y - line2_p1.y};
                const double dot = vec.x * line2_vec.x + vec.y * line2_vec.y;
                return dot / line2_length_sq;
                };

                const double lambda1 = computeLambda(line1_p1);
                const double lambda2 = computeLambda(line1_p2);

                // 判断两个投影参数是否都在[0,1]区间外
                const bool all_outside = 
                (lambda1 < 0 || lambda2 < 0) ||  // 两个投影在起点延长线侧
                (lambda1 > 1 || lambda2 > 1);    // 两个投影在终点延长线侧

                return all_outside;
        }     

} // end of namespace


// RCLCPP_COMPONENTS_REGISTER_NODE(line_path_compare::LinePathCompare)


