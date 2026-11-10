#include "nav2_planner.hpp"

Nav2Planner::Nav2Planner() : Node("nav2_planner")
{
    // client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "/compute_path_to_pose");
    client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(this, "/compute_path_through_poses");
    generate_path_srv_ = this->create_service<autonomous_navigation_msgs::srv::PathGenerator>("generate_path", std::bind(&Nav2Planner::generate_path_cb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Generating path");
}

PathType Nav2Planner::stringToPathType(const int & type)
{
    if (type == 0) return PathType::SQUARE;
    if (type == 1) return PathType::CIRCLE;
    if (type == 2) return PathType::SPIRAL;
    if (type == 3) return PathType::SERPENTINE;
    if (type == 4) return PathType::LINE;

    throw std::runtime_error("Unknown path type: " + std::to_string(type));
}

void Nav2Planner::waypointGenerate()
{   
    switch (path_type_)
    {
        case PathType::SQUARE:
        {
            squareGenerate();
            break;
        }
        case PathType::CIRCLE:
        {
            circleGenerate();
            break;
        }
        case PathType::SPIRAL:
        {
            spiralGenerate();
            break;
        }
        case PathType::SERPENTINE:
        {
            serpentineGenerate();
            break;
        }
        case PathType::LINE:
        {
            lineGenerate();
            break;
        }
    }
}

void Nav2Planner::squareGenerate()
{
    // Generate a square trajectory in the middle of the under side
    double L = path_params_.side;
    double d = path_params_.delta;
    double h = L/2;
    double dz = path_params_.dz;

    for (double y = 0; y <= h; y += d)
        wp_.push_back({0,y,dz});
    for (double x = d; x <= L; x += d)
        wp_.push_back({x,h,dz});
    for (double y = h-d; y >= -h; y -= d)
        wp_.push_back({L,y,dz});
    for (double x = L-d; x >= 0; x -= d)
        wp_.push_back({x,-h,dz});
    for (double y = -h+d; y <= 0; y += d)
        wp_.push_back({0,y,dz});
}

void Nav2Planner::circleGenerate()
{
    // Generate a circle trajectory with the center in (r,0)
    double r = path_params_.radius;
    double dz = path_params_.dz;

    int N = std::ceil((2*M_PI*r)/path_params_.delta);

    for (int i=0; i<=N; i++)
    {
        double theta = 2*M_PI*i/N;
        wp_.push_back({r-r*cos(theta), r*sin(theta), dz});
    }
}

void Nav2Planner::spiralGenerate()
{
    // Generate a spiral trajectory with the center in (r,0)
    double r = path_params_.radius;
    double dz = path_params_.dz;
    int turns = path_params_.turns;

    double length = turns*2*M_PI*r;
    int N = std::ceil(length/path_params_.delta);

    for (int i=0; i<=N; i++)
    {
        double theta = turns*2*M_PI*i/N;
        wp_.push_back({r - r*cos(theta), r*sin(theta), dz*theta/(2*M_PI)});
    }
}

void Nav2Planner::serpentineGenerate()
{
    // Clear previous waypoints
    wp_.clear();

    double w = -path_params_.dz; // Vertical depth (positive value)
    double d = path_params_.delta; // Step discretization
    int turns = path_params_.turns;
    double h = path_params_.side;  // Horizontal spacing between vertical legs

    double current_y = 0.0;

    for (int i = 0; i < turns; ++i)
    {
        if (i % 2 == 0)
        {
            // Even turns: Go DOWN (from Z = 0 to Z = -w)
            for (double z = 0.0; z >= -w; z -= d)
            {
                wp_.push_back({0.0, current_y, z});
            }

            // Move RIGHT to the next leg position (if not the last turn)
            if (i < turns - 1)
            {
                double target_y = current_y + h;
                for (double y = current_y + d; y <= target_y; y += d)
                {
                    wp_.push_back({0.0, y, -w});
                }
                current_y = target_y;
            }
        }
        else
        {
            // Odd turns: Go UP (from Z = -w to Z = 0)
            for (double z = -w; z <= 0.0; z += d)
            {
                wp_.push_back({0.0, current_y, z});
            }

            // Move RIGHT to the next leg position (if not the last turn)
            if (i < turns - 1)
            {
                double target_y = current_y + h;
                for (double y = current_y + d; y <= target_y; y += d)
                {
                    wp_.push_back({0.0, y, 0.0});
                }
                current_y = target_y;
            }
        }
    }
}

void Nav2Planner::lineGenerate()
{
    double L = path_params_.side;
    double d = path_params_.delta;
    // double dz = path_params_.dz;

    for (double x = 0; x >= -L; x -= d)
        wp_.push_back({0.0,0.0,x});
}

void Nav2Planner::generate_path_cb(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<autonomous_navigation_msgs::srv::PathGenerator::Request> request)
{
    wp_.clear();
    waypointGenerate();
    RCLCPP_INFO(this->get_logger(), "generate_path_cb CALLED");
    
    if (!client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        auto response = std::make_shared<autonomous_navigation_msgs::srv::PathGenerator::Response>();
        response->success = false;
        response->message = "Action server not available";
        generate_path_srv_->send_response(*header, *response);
        return;
    }

     path_params_.delta = request->delta;
    path_params_.side = request->side;
    path_params_.radius = request->radius;
    path_params_.dz = request->dz;
    path_params_.turns = request->turns;
    path_params_.depth = request->depth;
    path_params_.points = request->points;
    path_type_ = stringToPathType(request->path_type);
    RCLCPP_INFO(this->get_logger(), "%s", path_type_ == PathType::SQUARE ? "SQUARE" : path_type_ == PathType::CIRCLE ? "CIRCLE" : path_type_ == PathType::SPIRAL ? "SPIRAL" : path_type_ == PathType::SERPENTINE ? "SERPENTINE" : path_type_ == PathType::LINE ? "LINE" : "UNKNOWN");

    // auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    auto goal_msg = nav2_msgs::action::ComputePathThroughPoses::Goal();

    goal_msg.goals.clear();
    goal_msg.goals.reserve(wp_.size());
    // Converts the waypoints into posestamped
    for (const auto & w : wp_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();

        pose.pose.position.x = w.x;
        pose.pose.position.y = w.y;
        pose.pose.position.z = 0.0;

        goal_msg.goals.push_back(pose);
    }

    //especificar qual planner usar de acordo com o id no nav2_params.yaml
    goal_msg.planner_id = "GridBased";

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions();

    send_goal_options.result_callback = [this, header](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result) {
        auto response = std::make_shared<autonomous_navigation_msgs::srv::PathGenerator::Response>();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Trajectory generated with %zu waypoints", result.result->path.poses.size());
            response->success = true;
            response->message = "Path generated successfully";
            response->plan = result.result->path;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate the trajectory");
            response->success = false;
            response->message = "Failed to generate the trajectory";
        }
         generate_path_srv_->send_response(*header, *response);
    };
    client_->async_send_goal(goal_msg, send_goal_options);
}

// void Nav2Planner::generate_path_cb(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<autonomous_navigation_msgs::srv::Nav2Planner::Request> request)
// {
//     RCLCPP_INFO(this->get_logger(), "generate_path_cb CALLED");
    
//     if (!client_->action_server_is_ready()) {
//         RCLCPP_ERROR(this->get_logger(), "Action server not available");
//         auto response = std::make_shared<autonomous_navigation_msgs::srv::Nav2Planner::Response>();
//         response->success = false;
//         response->message = "Action server not available";
//         generate_path_srv_->send_response(*header, *response);
//         return;
//     }

//     // auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
//     auto goal_msg = nav2_msgs::action::ComputePathThroughPoses::Goal();
//     goal_msg.goals = request->goals;
//     goal_msg.start = request->start;
//     goal_msg.use_start = false;
//     //especificar qual planner usar de acordo com o id no nav2_params.yaml
//     goal_msg.planner_id = "GridBased";

//     auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions();

//     send_goal_options.result_callback = [this, header](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result) {
//         auto response = std::make_shared<autonomous_navigation_msgs::srv::Nav2Planner::Response>();
//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//             RCLCPP_INFO(this->get_logger(), "Trajectory generated with %zu waypoints", result.result->path.poses.size());
//             response->success = true;
//             response->message = "Path generated successfully";
//             response->plan = result.result->path;
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Failed to generate the trajectory");
//             response->success = false;
//             response->message = "Failed to generate the trajectory";
//         }
//          generate_path_srv_->send_response(*header, *response);
//     };
//     client_->async_send_goal(goal_msg, send_goal_options);
// }

// void Nav2Planner::send_goal()
// {
//     if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
//         RCLCPP_ERROR(this->get_logger(), "Action server not available");
//         return;
//     }

//     auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
//     goal_msg.goal.header.frame_id = "map";
//     goal_msg.goal.header.stamp = this->now();
//     goal_msg.goal.pose.position.x = 2.0;
//     goal_msg.goal.pose.position.y = 1.5;
//     goal_msg.goal.pose.orientation.w = 1.0;
//     //especificar qual planner usar de acordo com o id no nav2_params.yaml
//     goal_msg.planner_id = "GridBased";

//     auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

//     send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result) {
//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//             auto path = result.result->path;
//             RCLCPP_INFO(this->get_logger(), "Trajectory generated with %zu waypoints", path.poses.size());
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Failed to generate the trajectory");
//         }
//     };
//     client_->async_send_goal(goal_msg, send_goal_options);
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Planner>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    // node->send_goal();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}