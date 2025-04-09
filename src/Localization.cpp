#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    odometry_.pose.pose.position.x = 0.5;
    odometry_.pose.pose.position.y = 2.0;

    // add code here

  // Subscriber pro zprávy joint_states
  joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

  // Publisher pro odometrické zprávy
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
  auto current_time = this->get_clock()->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  double left_wheel_vel = 0.0;
  double right_wheel_vel = 0.0;

  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] == "wheel_left_joint") {
      left_wheel_vel = msg.velocity[i];
      if(!msg.velocity[i])
        left_wheel_vel = 0.0;
  } else if (msg.name[i] == "wheel_right_joint") {

    right_wheel_vel = msg.velocity[i];
    if(!msg.velocity[i])
        left_wheel_vel = 0.0;
      
  }
    
  }

  // Aktualizace odometrie na základě rychlostí kol
  if(msg.velocity.empty())
    return;
  updateOdometry(left_wheel_vel, right_wheel_vel, dt);
  publishOdometry();
  publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
  // Parametry robota
  double wheel_base = robot_config::HALF_DISTANCE_BETWEEN_WHEELS*2;  // Vzdálenost mezi koly
  double radius = robot_config::WHEEL_RADIUS;  // Poloměr kola

  // Vypočítáme lineární a úhlovou rychlost
  double linear_velocity = radius * (left_wheel_vel + right_wheel_vel) / 2.0;
  double angular_velocity = radius * (right_wheel_vel - left_wheel_vel) / wheel_base;

  // Získání aktuální orientace robota
  tf2::Quaternion q;
  tf2::fromMsg(odometry_.pose.pose.orientation, q);

  // Získání roll, pitch, theta (yaw)
  double roll, pitch, theta;
  tf2::Matrix3x3(q).getRPY(roll, pitch, theta);

  // Aktualizace pozice robota (x, y) a orientace (theta)
  double delta_x = linear_velocity * dt * cos(theta);
  double delta_y = linear_velocity * dt * sin(theta);
  odometry_.pose.pose.position.x += delta_x;
  odometry_.pose.pose.position.y += delta_y;

  // Aktualizace orientace (yaw)
  double delta_theta = angular_velocity * dt;
  q.setRPY(0, 0, theta + delta_theta);
  odometry_.pose.pose.orientation = tf2::toMsg(q);

  // Aktualizace lineární a úhlové rychlosti
  odometry_.twist.twist.linear.x = linear_velocity;
  odometry_.twist.twist.angular.z = angular_velocity;
}

void LocalizationNode::publishOdometry() {
  // Publikování zprávy o odometrii na téma "odom"
  odometry_.header.stamp = this->get_clock()->now();
  odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
  // Vytvoření a publikování transformace mezi "odom" a "base_link"
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->get_clock()->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = odometry_.pose.pose.position.x;
  transform_stamped.transform.translation.y = odometry_.pose.pose.position.y;
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation = odometry_.pose.pose.orientation;
  /*if (std::isnan(odometry_.pose.pose.orientation.x) || 
  std::isnan(odometry_.pose.pose.orientation.y) || 
  std::isnan(odometry_.pose.pose.orientation.z) || 
  std::isnan(odometry_.pose.pose.orientation.w)) {
  RCLCPP_WARN(get_logger(), "Ignoring transform due to NaN in quaternion.");
  return;  // Pokud je quaternion na NaN, ignorujte publikování
  }*/
  // Publikování transformace
  tf_broadcaster_->sendTransform(transform_stamped);
}
