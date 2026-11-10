#ifndef NAV2_PLANNER_HPP
#define NAV2_PLANNER_HPP

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "autonomous_navigation_msgs/srv/path_generator.hpp"

//library to convert waypoints into posestamped
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

enum class PathType
{
    SQUARE,
    CIRCLE,
    SPIRAL,
    SERPENTINE,
    LINE
};

class Nav2Planner : public rclcpp::Node
{
public:
    Nav2Planner();
    void send_goal();
private:
    // rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr client_;
    void generate_path_cb(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<autonomous_navigation_msgs::srv::PathGenerator::Request> request);
    rclcpp::Service<autonomous_navigation_msgs::srv::PathGenerator>::SharedPtr generate_path_srv_;

    struct waypoint
    {
        double x;
        double y;
        double z;
    };
    std::vector<waypoint> wp_;
    struct PathParams
    {
        double delta{0.2};
        double side{2.0};
        double radius{1.0};
        double dz{-1.0};
        int turns{4};
        int depth{4};
        int points{50};
    };
    PathParams path_params_;
    PathType path_type_;

    PathType stringToPathType(const int & type);
    void waypointGenerate();
    void squareGenerate();
    void circleGenerate();
    void spiralGenerate();
    void serpentineGenerate();
    void lineGenerate();

};

#endif // NAV2_PLANNER_HPP