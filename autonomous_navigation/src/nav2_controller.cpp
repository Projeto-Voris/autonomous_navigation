#include "nav2_controller.hpp"

Nav2Controller::Nav2Controller() : Node("nav2_controller")
{
    this->declare_parameter<std::string>("frame_id", "odom");

    this->declare_parameter<double>("delta", 0.2);
    this->declare_parameter<double>("side", 2.0);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<double>("dz", -1.0);
    this->declare_parameter<int>("turns", 4);
    this->declare_parameter<int>("depth", 4);
    this->declare_parameter<int>("points", 50);
    this->declare_parameter<int>("path_type", 0);

    this->declare_parameter<double>("duct_x", 3.0);
    this->declare_parameter<double>("duct_y", 1.5);
    this->declare_parameter<double>("duct_z", -1.0);
    
    this->declare_parameter<bool>("debug_mode", false);

    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("duct_x", duct_position_.pdx);
    this->get_parameter("duct_y", duct_position_.pdy);
    this->get_parameter("duct_z", duct_position_.pdz);

    if(this->get_parameter("debug_mode").as_bool())
    {
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
        slam_file_.open("/home/biancarosa/Documents/logs_auv/slam_pose.txt");
        odom_file_.open("/home/biancarosa/Documents/logs_auv/odometry.txt");
        wp_file_.open("/home/biancarosa/Documents/logs_auv/wp_file_.txt");
        time_.open("/home/biancarosa/Documents/logs_auv/time_.txt");
    }

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("transformed_path", 10);

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    generate_trajectory_client = this->create_client<autonomous_navigation_msgs::srv::PathGenerator>("generate_path", rmw_qos_profile_services_default, cb_group_);
    mavros_arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", rmw_qos_profile_services_default, cb_group_);
    mavros_set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", rmw_qos_profile_services_default, cb_group_);
    start_trajectory_srv = this->create_service<std_srvs::srv::SetBool>("start_trajectory", std::bind(&Nav2Controller::start_trajectory_cb, this, std::placeholders::_1, std::placeholders::_2));
    nav2_follow_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "/follow_path");

    slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 10, std::bind(&Nav2Controller::slam_cb, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&Nav2Controller::state_cb, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("local_path", 10, std::bind(&Nav2Controller::transformPath, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Nav2Controller::pathFollower, this), cb_group_timer_);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_sub_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", qos, std::bind(&Nav2Controller::odom_cb, this, std::placeholders::_1), sub_options);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m){ last_cmd_ = *m; last_cmd_time_ = now();});
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped",10);
    vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Nav2Controller::publishVelocity, this), cb_group_timer_);
}

Nav2Controller::~Nav2Controller()
{
    if(this->get_parameter("debug_mode").as_bool())
    {
        slam_file_.close();
        odom_file_.close();
        wp_file_.close();
        time_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Disarming the robot before shutdown");
    if(set_arm(false)){
        rclcpp::shutdown();
    }

}

void Nav2Controller::start_trajectory_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        RCLCPP_INFO(this->get_logger(), "Starting trajectory following");

        follow_path = false;
        goal_sent_ = false;
        segments_.clear();
        seg_idx_ = 0;
        path_transformed_.clear();

        last_time_ = this->now();
        time0_=this->now();
        last_cmd_time_ = this->now();
        set_origin_ = false; // Reset the origin to get the current position as the new origin
        auto req = std::make_shared<autonomous_navigation_msgs::srv::PathGenerator::Request>();
        req->delta = this->get_parameter("delta").as_double();
        req->side = this->get_parameter("side").as_double();
        req->radius = this->get_parameter("radius").as_double();
        req->dz = this->get_parameter("dz").as_double();
        req->turns = this->get_parameter("turns").as_int();
        req->depth = this->get_parameter("depth").as_int();
        req->points = this->get_parameter("points").as_int();
        req->path_type = this->get_parameter("path_type").as_int();

        auto future = generate_trajectory_client->async_send_request(req);

        // Wait for the future safely (since the callback group is Reentrant)
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            response->success = false;
            response->message = "Timeout calling generate_path";
            return;
        }
        auto result = future.get();
        if (!result->success) {
            response->success = false;
            response->message = "Failed to generate trajectory: " + result->message;
            return;
        }
        trajectory_index_ = 0;
        follow_path = true;
        response->success = true;
        response->message = "Trajectory following started";
    } else {
        follow_path = false;
        goal_sent_ = false;
        response->success = true;
        response->message = "Trajectory following stopped";
    }
}

void Nav2Controller::state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg)
{
    connected_ = msg->connected;
    armed_ = msg->armed;
    mode_ = msg->mode;
}

bool Nav2Controller::set_arm(bool arm)
{
    if (!connected_) {
        RCLCPP_ERROR(this->get_logger(), "ROV not connected, cannot change arm state");
        return false; 
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    // call mavros/cmd/arming
    if(mavros_arm_client_->service_is_ready()) {
        mavros_arm_client_->async_send_request(request);
        RCLCPP_INFO_ONCE(this->get_logger(), arm ? "ROV armed" : "ROV disarmed");
        return true;
    }
    return false;
}

bool Nav2Controller::set_mode(const std::string & mode)
{
    if(!connected_) {
        RCLCPP_ERROR(this->get_logger(), "ROV not connected, cannot change mode");
        return false; 
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    if(mavros_set_mode_client_->service_is_ready()) {
        mavros_set_mode_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Robot change mode to: ");
        return true;
    }
    return false;
}

void Nav2Controller::publishPath()
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = frame_id_;
    for (const auto& p : path_transformed_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = p.z;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
}

void Nav2Controller::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
    // position (position and orientation)
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;

    // velocity (linear and angular)
    current_vel_ = msg->twist.twist;

    // RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,"x = %.3f",current_pose_.pose.position.x);

    if (!set_origin_)
    {
        origin_x_ = current_pose_.pose.position.x;
        origin_y_ = current_pose_.pose.position.y;
        origin_z_ = current_pose_.pose.position.z;

        // Transformation to the start position
        T_map_start.setOrigin(tf2::Vector3(origin_x_,origin_y_,origin_z_));
        double initial_yaw = tf2::getYaw(current_pose_.pose.orientation);

        // Construct a pure Yaw quaternion (Roll = 0, Pitch = 0, Yaw = initial_yaw)
        tf2::Quaternion q_yaw_only;
        q_yaw_only.setRPY(0.0, 0.0, initial_yaw);

        // Assign pure Yaw rotation to T_map_start
        T_map_start.setRotation(q_yaw_only);

        T_start_map_ = T_map_start.inverse();

        set_origin_ = true;

        RCLCPP_INFO(get_logger(),"Origin: %.2f %.2f %.2f",T_start_map_.getOrigin().x(),T_start_map_.getOrigin().y(),T_start_map_.getOrigin().z());

        RCLCPP_INFO_ONCE(this->get_logger(), "origin_x:%4f | origin_y:%4f | origin_z:%4f", origin_x_, origin_y_,origin_z_);
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "header_frame_id:%s ", current_pose_.header.frame_id.c_str());

    if(this->get_parameter("debug_mode").as_bool())
    {
        odom_file_ << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << '\n';
    }   
}

void Nav2Controller::slam_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
    if(this->get_parameter("debug_mode").as_bool())
    {
        slam_file_ << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << '\n';
    }
}

void Nav2Controller::transformPath(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
    path_transformed_.clear();
    segments_.clear();
    seg_idx_ = 0;

    for (const auto &p : msg->poses)
    {
        // wp_file_ << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << "\n";
        // wp_file_ << std::endl;
        tf2::Vector3 pt(p.pose.position.x, p.pose.position.y, p.pose.position.z);

        // tf2::Vector3 pt_map = T_map_start * pt;
        tf2::Vector3 pt_map(pt.x() + origin_x_, pt.y() + origin_y_, pt.z() + origin_z_);

        path_transformed_.push_back({
            pt_map.x(),
            pt_map.y(),
            pt_map.z()

        });
    }

    if (!path_transformed_.empty())
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;

        for (const auto &p : path_transformed_)
        {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }

        size_t n = path_transformed_.size();
        duct_position_.pdx = sum_x / n;
        duct_position_.pdy = sum_y / n;
        duct_position_.pdz = sum_z / n;

        RCLCPP_INFO(this->get_logger(), 
            "Calculated Path Center / Duct Position: [x: %.3f, y: %.3f, z: %.3f]", 
            duct_position_.pdx, duct_position_.pdy, duct_position_.pdz);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received path is empty. Duct position was not updated.");
    }
    if(this->get_parameter("debug_mode").as_bool())
    {
        for (const auto &p : path_transformed_)
        {
            wp_file_ << p.x << " " << p.y << " " << p.z << "\n";
        }
        wp_file_ << std::endl;
    }
    publishPath();
}

void Nav2Controller::feedback_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Received feedback");
}

void Nav2Controller::Segments(double min_seg)
{
    segments_.clear();
    seg_idx_ = 0;
    if (path_transformed_.size() < 2) {
        return;
    }

    size_t start = 0;
    for (size_t i = 1; i < path_transformed_.size(); ++i) {
        const auto & a = path_transformed_[start];
        const auto & b = path_transformed_[i];
        if (std::hypot(b.x - a.x, b.y - a.y) >= min_seg) {
            segments_.emplace_back(start,i);
            start = i;
        }
    }
    if (start + 1 < path_transformed_.size()) {
        const auto & a = path_transformed_[start];
        const auto & b = path_transformed_.back();
        if (std::hypot(b.x - a.x, b.y - a.y) > 0.25) {
            segments_.emplace_back(start, path_transformed_.size() - 1);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Path split into %zu segments", segments_.size());
}

void Nav2Controller::pathFollower()
{
    if (!follow_path) { 
        return;
    }

    if (goal_sent_) {
        return;
    }

    if (path_transformed_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for local_path...");
        return;
    } else {
        if (!connected_) {
            RCLCPP_ERROR(this->get_logger(), "ROV not connected");
            return;
        }
        if (mode_!= "GUIDED") {
            set_mode("GUIDED");
            return;
        }
        if (!armed_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *  this->get_clock(), 500, "ROV not armed, arming now");
            set_arm(true);
            return;
        }
    }

    if (!nav2_follow_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "FollowPath server not ready");
        return;
    }

    nav2_msgs::action::FollowPath::Goal goal_msg;
    goal_msg.path.header.frame_id = frame_id_;
    goal_msg.path.header.stamp = now();

    if (segments_.empty()) {
        Segments(0.8);
        if (segments_.empty()) {
            follow_path = false;
            return;
        }
    }

    if (seg_idx_ >= segments_.size()) {
        return;
    }

    const auto [s, e] = segments_[seg_idx_];
    double yaw = 0.0;
    for (size_t i = s; i <= e; ++i) {
        const auto & w = path_transformed_[i];
        if (i < e) {
            const auto & nx = path_transformed_[i+1];
            yaw = std::atan2(nx.y - w.y, nx.x - w.x);
        }
        // Converts the waypoints into posestamped
        geometry_msgs::msg::PoseStamped pose;
        pose.header = goal_msg.path.header;

        pose.pose.position.x = w.x;
        pose.pose.position.y = w.y;
        pose.pose.position.z = w.z;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        goal_msg.path.poses.push_back(pose);
    }
    goal_msg.controller_id = "FollowPath";
    goal_msg.goal_checker_id = "goal_checker";

    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions opts;
    opts.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr gh)
    {
        if (!gh) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            goal_sent_ = false;
            follow_path = false;
        }
    };
    opts.feedback_callback = std::bind(&Nav2Controller::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult & r) {
        goal_sent_ = false;
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
            if (++seg_idx_ < segments_.size()) {
                RCLCPP_INFO(this->get_logger(), "Segment %zu/%zu done", seg_idx_, segments_.size());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Trajectory completed");}
        else {
            RCLCPP_WARN(this->get_logger(), "FollowPath ended with code %d", (int)r.code);}
        follow_path = false;
        segments_.clear();
        seg_idx_ = 0;
    };
    
    nav2_follow_client_->async_send_goal(goal_msg, opts);
    goal_sent_ = true;
}

void Nav2Controller::publishVelocity()
{
    geometry_msgs::msg::Twist out;

    const bool fresh = follow_path && (now() - last_cmd_time_).seconds() < 0.5;
    if (fresh) {
        out.linear.x  = last_cmd_.linear.x;
        out.linear.y  = last_cmd_.linear.y;
        out.angular.z = last_cmd_.angular.z;   // faltava
    }

    if (follow_path) {
        double err = target_depth_ - current_pose_.pose.position.z;
        out.linear.z = std::clamp(1.0 * err, -0.3, 0.3);
    }

    vel_pub_->publish(out);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Controller>(); 
    // Nav2Controller::instance = node;
    // std::signal(SIGINT, Nav2Controller::sigintHandler);

    // Substitua o rclcpp::spin(node) por MultiThreadedExecutor:
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}