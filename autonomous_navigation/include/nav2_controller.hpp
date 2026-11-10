#ifndef NAV2_CONTROLLER_HPP
#define NAV2_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include "rosidl_runtime_cpp/traits.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <autonomous_navigation_msgs/srv/path_generator.hpp>
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <csignal>

#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Nav2Controller : public rclcpp::Node
{
public:
    Nav2Controller();
    ~Nav2Controller();

private:
    bool connected_{};
    bool armed_{};
    bool set_origin_ = false;
    bool follow_path = false;
    std::string mode_ = "";
    double gain_;
    bool goal_sent_ = false;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    void state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    void transformPath(const nav_msgs::msg::Path::ConstSharedPtr & msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_sub_;
    void slam_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_trajectory_srv;
    void start_trajectory_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    void feedback_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback);

    // Current robot state
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_vel_;
    geometry_msgs::msg::PoseStamped pose_odom_;
    rclcpp::Time last_time_;
    rclcpp::Time time0_;
    // nav_msgs::msg::Path current_path_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Client<autonomous_navigation_msgs::srv::PathGenerator>::SharedPtr generate_trajectory_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr mavros_arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mavros_set_mode_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr nav2_follow_client_;   

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool set_arm(bool arm);
    bool set_mode(const std::string & mode);
    void publishPath();
    void pathFollower();

    struct DuctPosition
    {
        double pdx{3.0};
        double pdy{1.5};
        double pdz{-1.0};
    };
    DuctPosition duct_position_;

    rclcpp::TimerBase::SharedPtr timer_tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2::Transform T_start_map_;
    tf2::Transform T_map_start;
    //tf2::Quaternion lookAtTheDuct(const geometry_msgs::msg::Point& robot_position);
    std::vector<tf2::Quaternion> trajectory_orientation_;

    double origin_x_;
    double origin_y_;
    double origin_z_;

    struct waypoint
    {
        double x;
        double y;
        double z;
    };
    
    std::vector<waypoint> path_transformed_;
    size_t trajectory_index_;
    std::string frame_id_;

    std::ofstream slam_file_;
    std::ofstream odom_file_;
    std::ofstream wp_file_;
    std::ofstream time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_cmd_time_;
    double target_depth_{0.0};
    rclcpp::TimerBase::SharedPtr vel_timer_;
    void publishVelocity();

    //inicialização de variaveis que dividem a trajetória em segmentos para o DWB seguir
    std::vector<std::pair<size_t,size_t>> segments_; // [inicial, fim]
    size_t seg_idx_ = 0;
    void Segments(double min_seg);
};

#endif // NAV2_CONTROLLER_HPP