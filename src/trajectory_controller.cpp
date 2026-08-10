#include "trajectory_controller.hpp"

//Purpose: Follow a path using RC override commands

TrajectoryController::TrajectoryController() : Node("trajectory_controller")
{
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("duct_x", 3.0);
    this->declare_parameter<double>("duct_y", 1.5);
    this->declare_parameter<double>("duct_z", -1.0);
    this->declare_parameter<double>("kp", 2.0);
    this->declare_parameter<double>("ki", 1.0);

    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("duct_x", duct_position_.pdx);
    this->get_parameter("duct_y", duct_position_.pdy);
    this->get_parameter("duct_z", duct_position_.pdz);
    kp = this->get_parameter("kp").as_double();

    slam_file_.open("/home/biancarosa/Documents/logs_auv/slam_pose.txt");
    odom_file_.open("/home/biancarosa/Documents/logs_auv/odometry.txt");
    wp_file_.open("/home/biancarosa/Documents/logs_auv/wp_file_.txt");
    alt_file_.open("/home/biancarosa/Documents/logs_auv/alt_file_.txt");

    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("transformed_path", 10);

    // In your TrajectoryController constructor:
    rclcpp::CallbackGroup::SharedPtr cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr cb_group_timer = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    mavros_arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", rmw_qos_profile_services_default, cb_group_);
    mavros_set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", rmw_qos_profile_services_default, cb_group_);
    start_trajectory_srv = this->create_service<std_srvs::srv::SetBool>("start_trajectory", std::bind(&TrajectoryController::start_trajectory_cb, this, std::placeholders::_1, std::placeholders::_2));

    slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", 10, std::bind(&TrajectoryController::slam_cb, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&TrajectoryController::state_cb, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("local_path", 10, std::bind(&TrajectoryController::transformPath, this, std::placeholders::_1));
    alt_sub_ = this->create_subscription<mavros_msgs::msg::VfrHud>("/mavros/vfr_hud", 10, std::bind(&TrajectoryController::alt_cb, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajectoryController::pathFollower, this), cb_group_timer);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", qos, std::bind(&TrajectoryController::odom_cb, this, std::placeholders::_1));
    
    gain_ = 0.3;
    trajectory_index_ = 0;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void TrajectoryController::start_trajectory_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    follow_path = request->data;
    if (follow_path) {
        RCLCPP_INFO(this->get_logger(), "Starting trajectory following");
        // trajectory_index_ = 0; // Reset the trajectory index when starting
    } else {
        RCLCPP_INFO(this->get_logger(), "Stopping trajectory following");
        publish_rc(0.0, 0.0, 0.0, 0.0); // Stop the robot
    }
    response->success = true;
    response->message = follow_path ? "Trajectory following started" : "Trajectory following stopped";
}
void TrajectoryController::state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg)
{
    connected_ = msg->connected;
    armed_ = msg->armed;
    mode_ = msg->mode;
}

bool TrajectoryController::set_arm(bool arm)
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    // call mavros/cmd/arming
    if(mavros_arm_client_->service_is_ready()) {
        mavros_arm_client_->async_send_request(request);
        RCLCPP_INFO_ONCE(this->get_logger(), "Robot Armed");
        return true;
    }
    return false;
}

bool TrajectoryController::disarm()
{
    if (!connected_) {
        return false; 
    }
    if (armed_) {
        bool result = set_arm(false);
        if (result) {
        RCLCPP_INFO_ONCE(get_logger(), "Robot disarmed");
        }
        return result;
    }

    return true;
}

bool TrajectoryController::set_mode(const std::string & mode)
{
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
        tf2::Quaternion q;
        tf2::fromMsg(current_pose_.pose.orientation, q);
        T_map_start.setRotation(q);
        T_start_map_ = T_map_start.inverse();

        set_origin_ = true;

        RCLCPP_INFO(get_logger(),"Origin: %.2f %.2f %.2f",T_start_map_.getOrigin().x(),T_start_map_.getOrigin().y(),T_start_map_.getOrigin().z());

        RCLCPP_INFO_ONCE(this->get_logger(), "origin_x:%4f | origin_y:%4f | origin_z:%4f", origin_x_, origin_y_,origin_z_);
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "header_frame_id:%s ", current_pose_.header.frame_id.c_str());

    odom_file_ << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << '\n';
}

void TrajectoryController::slam_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
    slam_file_ << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << '\n';
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

    for (const auto &p : path_transformed_)
    {
        wp_file_ << p.x << " " << p.y << " " << p.z << "\n";
    }
    wp_file_ << std::endl;
    publishPath();
}

void TrajectoryController::alt_cb(const mavros_msgs::msg::VfrHud::ConstSharedPtr & msg)
{
    alt_file_ << msg->altitude << '\n';
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
        RCLCPP_INFO(this->get_logger(), "Trajectory following is disabled");
        return;}

    if (trajectory_index_>= path_transformed_.size()) {
        RCLCPP_INFO_ONCE(get_logger(), "Trajectory completed");
        disarm();
        publish_rc(0.0, 0.0, 0.0, 0.0);
        return;
    } else {
        RCLCPP_INFO_ONCE(get_logger(), "Starting follow waypoints");
        if (!connected_) {
            RCLCPP_INFO_ONCE(get_logger(), "Robot not connected");
            return;
        }
        if (mode_!= "ALT_HOLD") {
            set_mode("ALT_HOLD");
            return;
        }
        if (!armed_) {
            RCLCPP_INFO_ONCE(get_logger(), "Robot not armed");
            set_arm(true);
            return;
        }
    }

    if (path_transformed_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory is empty");
        return;
    }
    waypoint target = path_transformed_[trajectory_index_];
    tf2::Quaternion q_desired = lookAtTheDuct(current_pose_.pose.position);

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

    double forward_cmd = kp*error_forward;
    double lateral_cmd = kp*error_lateral;
    double depth_cmd = kp*error_z;
    double yaw_cmd = kp*yaw;

    // Euclidean distance
    double distance = std::hypot(error_x,error_y, error_z);
    float threshold = 0.3;

    if (distance < threshold)
    {
        trajectory_index_++;
    }

    publish_rc(forward_cmd, lateral_cmd, depth_cmd, yaw_cmd);
}

std::shared_ptr<TrajectoryController> TrajectoryController::instance = nullptr;
void TrajectoryController::sigintHandler(int)
{
    if (instance){
        instance->disarm();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryController>(); 
    TrajectoryController::instance = node;
    std::signal(SIGINT, TrajectoryController::sigintHandler);
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}