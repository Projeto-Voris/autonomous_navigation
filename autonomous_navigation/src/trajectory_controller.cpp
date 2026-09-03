#include "trajectory_controller.hpp"

//Purpose: Follow a path using RC override commands

TrajectoryController::TrajectoryController() : Node("trajectory_controller")
{
    this->declare_parameter<std::string>("frame_id", "map");

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
    this->declare_parameter<double>("kp", 0.54);
    this->declare_parameter<double>("ki", 0.162);
    
    this->declare_parameter<bool>("debug_mode", false);

    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("duct_x", duct_position_.pdx);
    this->get_parameter("duct_y", duct_position_.pdy);
    this->get_parameter("duct_z", duct_position_.pdz);
    kp = this->get_parameter("kp").as_double();
    ki = this->get_parameter("ki").as_double();

    if(this->get_parameter("debug_mode").as_bool())
    {
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
        slam_file_.open("/home/biancarosa/Documents/logs_auv/slam_pose.txt");
        odom_file_.open("/home/biancarosa/Documents/logs_auv/odometry.txt");
        wp_file_.open("/home/biancarosa/Documents/logs_auv/wp_file_.txt");
        alt_file_.open("/home/biancarosa/Documents/logs_auv/alt_file_.txt");
        control_.open("/home/biancarosa/Documents/logs_auv/control_.txt");
        time_.open("/home/biancarosa/Documents/logs_auv/time_.txt");
    }

    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("transformed_path", 10);

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    generate_trajectory_client = this->create_client<autonomous_navigation_msgs::srv::PathGenerator>("generate_path", rmw_qos_profile_services_default, cb_group_);
    mavros_arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", rmw_qos_profile_services_default, cb_group_);
    mavros_set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", rmw_qos_profile_services_default, cb_group_);
    start_trajectory_srv = this->create_service<std_srvs::srv::SetBool>("start_trajectory", std::bind(&TrajectoryController::start_trajectory_cb, this, std::placeholders::_1, std::placeholders::_2));


    slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10, std::bind(&TrajectoryController::slam_cb, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&TrajectoryController::state_cb, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("local_path", 10, std::bind(&TrajectoryController::transformPath, this, std::placeholders::_1));
    alt_sub_ = this->create_subscription<mavros_msgs::msg::VfrHud>("/mavros/vfr_hud", 10, std::bind(&TrajectoryController::alt_cb, this, std::placeholders::_1));
    
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajectoryController::pathFollower, this), cb_group_timer_);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_sub_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", qos, std::bind(&TrajectoryController::odom_cb, this, std::placeholders::_1), sub_options  );
    
    gain_ = 0.3;
    trajectory_index_ = 0;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

TrajectoryController::~TrajectoryController()
{
    if(this->get_parameter("debug_mode").as_bool())
    {
        slam_file_.close();
        odom_file_.close();
        wp_file_.close();
        alt_file_.close();
        control_.close();
        time_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Disarming the robot before shutdown");
    if(set_arm(false)){
        rclcpp::shutdown();
    }

}

void TrajectoryController::start_trajectory_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        RCLCPP_INFO(this->get_logger(), "Starting trajectory following");
        last_time_ = this->now();
        time0_=this->now();
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
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Trajectory generated successfully");
                rclcpp::sleep_for(std::chrono::milliseconds(2000)); // Wait for a short duration to ensure the path is published
                trajectory_index_ = 0;
                follow_path = request->data; // Start following the path
                response->success = true;
                response->message = "Trajectory generated and started successfully";
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to generate trajectory: %s", result->message.c_str());
                response->success = false;
                response->message = "Failed to generate trajectory: " + result->message;
            }
    } else {
        RCLCPP_INFO(this->get_logger(), "Stopping trajectory following");
        follow_path = request->data; 
        publish_rc(0.0, 0.0, 0.0, 0.0); // Stop the robot
    }
    response->success = true;
    response->message = follow_path ? "Trajectory following started" : "Trajectory following stopped";
    }
}
void TrajectoryController::state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg)
{
    connected_ = msg->connected;
    armed_ = msg->armed;
    mode_ = msg->mode;
}

bool TrajectoryController::set_arm(bool arm)
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

bool TrajectoryController::set_mode(const std::string & mode)
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

uint16_t TrajectoryController::map_pwm(float value, bool reverse, float threshold)
{
    if (std::abs(value) < threshold) {
        return static_cast<uint16_t>(std::clamp(1500, 1100, 1900));
    }
    if (reverse) value=-value;
    int pwm = 1500 + (value * 400.0 * gain_);
    return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
}

void TrajectoryController::publish_rc(float forward, float lateral, float depth, float yaw)
{
    mavros_msgs::msg::OverrideRCIn rc_msg;

    for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }
    rc_msg.channels[5 - 1] = map_pwm(forward,false, 0.1); // Forward 
    rc_msg.channels[6 - 1] = map_pwm(lateral,true, 0.1); // Lateral 
    rc_msg.channels[4 - 1] = map_pwm(yaw,true, 0.1); // Yaw
    rc_msg.channels[3 - 1] = map_pwm(depth,false, 0.1); // Depth

    // RCLCPP_INFO(this->get_logger(), "forw:%4d | lat:%4d | yaw:%4d | depth:%4d ", rc_msg.channels[5 - 1], rc_msg.channels[6 - 1], rc_msg.channels[4 - 1], rc_msg.channels[3 - 1]);

    rc_pub_->publish(rc_msg);
}

void TrajectoryController::publishPath()
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

void TrajectoryController::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
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

void TrajectoryController::slam_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
    if(this->get_parameter("debug_mode").as_bool())
    {
        slam_file_ << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << '\n';
    }
}

void TrajectoryController::transformPath(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
    path_transformed_.clear();

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

void TrajectoryController::alt_cb(const mavros_msgs::msg::VfrHud::ConstSharedPtr & msg)
{
    if(this->get_parameter("debug_mode").as_bool())
    {
        alt_file_ << msg->altitude << '\n';
    }
}

tf2::Quaternion TrajectoryController::lookAtTheDuct(const geometry_msgs::msg::Point& robot_position)
{
    // Process Gram-Schmidt to change the orientation of the vector to keep the x axis toward the duct

    // Vector of the duct position
    Eigen::Vector3d pduct(duct_position_.pdx, duct_position_.pdy, duct_position_.pdz);

    // Vector of the current pose of the ROV
    Eigen::Vector3d robot(robot_position.x, robot_position.y, robot_position.z);

    // Vector that point from the ROV toward the duct
    Eigen::Vector3d x_axis = (pduct - robot).normalized();

    // keep the z axis aligned with the world vertical direction 
    Eigen::Vector3d z_ref(0,0,1); 

    // remove component parallel to x - project z_ref onto plane perpendicular to x_axis
    Eigen::Vector3d z_axis = z_ref - z_ref.dot(x_axis)*x_axis;
    if (z_axis.norm() < 1e-3)
    {
        // x is almost parallel ro z_ref
        z_ref = Eigen::Vector3d(0,1,0);
        z_axis = z_ref - z_ref.dot(x_axis)*x_axis;
    }
    z_axis.normalize();
    // complete the orthonormal basis between x axis and z axis 
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    // calculete the matrix of rotate
    Eigen::Matrix3d R;
    R.col(0) = x_axis; 
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    // Convert to quartenion
    Eigen::Quaterniond q(R);
    tf2::Quaternion q_tf;
    q_tf.setValue(q.x(), q.y(), q.z(), q.w());

    return q_tf;
}

void TrajectoryController::pathFollower()
{

    if (!follow_path) { 
        return;}

    if (trajectory_index_>= path_transformed_.size()) {
        RCLCPP_INFO(this->get_logger(), "Trajectory completed");
        // disarm();
        publish_rc(0.0, 0.0, 0.0, 0.0);
        follow_path = false;
        return;
    } else {
        if (!connected_) {
            RCLCPP_ERROR(this->get_logger(), "ROV not connected");
            return;
        }
        if (mode_!= "ALT_HOLD") {
            set_mode("ALT_HOLD");
            return;
        }
        if (!armed_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *  this->get_clock(), 500, "ROV not armed, arming now");
            set_arm(true);
            return;
        }
    }

    if (path_transformed_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory is empty");
        return;
    }

    rclcpp::Time now = this->now();
    if (last_time_.nanoseconds() == 0)
    {
        last_time_ = now;
        return;
    }
    double dt = (now - last_time_).seconds();
    // double time = (now - time0_).seconds();

    waypoint target = path_transformed_[trajectory_index_];
    // tf2::Quaternion q_desired = lookAtTheDuct(current_pose_.pose.position);
    tf2::Quaternion q_desired(0.0, 0.0, 0.0, 1.0); // No rotation for now

    // Current Pose
    double current_x = current_pose_.pose.position.x;
    double current_y = current_pose_.pose.position.y;
    double current_z = current_pose_.pose.position.z;
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);

    double error_x = target.x - current_x;
    double error_y = target.y - current_y;
    double error_z = target.z - current_z;

    // calculte the orientation
    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose_.pose.orientation, q_current);
    tf2::Quaternion q_error = q_desired*q_current.inverse();
    q_error.normalize();

    // Obtain the yaw angle 
    tf2::Matrix3x3 m(q_error);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    // Rotation of the ROV
    double error_forward = std::cos(current_yaw)*error_x + std::sin(current_yaw)*error_y;
    double error_lateral = -std::sin(current_yaw)*error_x + std::cos(current_yaw)*error_y;
    
    // Integral control
    ef_integral_ += error_forward*dt;
    el_integral_ += error_lateral*dt;
    ed_integral_ += error_z*dt;
    ey_integral_ += yaw*dt;

    // Derivate control
    double ef_derivate_ = (error_forward - ef_last_)/dt;
    double el_derivate_ = (error_lateral - el_last_)/dt;
    double ed_derivate_ = (error_z - ed_last_)/dt;
    double ey_derivate_ = (yaw - ey_last_)/dt;

    // PID control
    forward_cmd = 2.4*error_forward + 2*ef_integral_ + 0.72*ef_derivate_;
    lateral_cmd = 3*error_lateral + 2.6*el_integral_ + 0.86*el_derivate_;
    depth_cmd = 6*error_z + 6*ed_integral_ + 1.5*ed_derivate_;
    yaw_cmd = 0.9*yaw + 0.9*ey_integral_ + 0.3*ey_derivate_;

    ef_last_ = error_forward;
    el_last_ = error_lateral;
    ed_last_ = error_z;
    ey_last_ = yaw;

    last_time_ = now;

    // Euclidean distance
    double distance = std::hypot(error_x,error_y, error_z);
    float threshold = 0.3;

    if (distance < threshold)
    {
        trajectory_index_++;
    }

    publish_rc(forward_cmd, lateral_cmd, depth_cmd, yaw_cmd);

    // control_ << depth_cmd << "\n";
    // time_ << time << "\n";
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryController>(); 
    // TrajectoryController::instance = node;
    // std::signal(SIGINT, TrajectoryController::sigintHandler);

    // Substitua o rclcpp::spin(node) por MultiThreadedExecutor:
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}