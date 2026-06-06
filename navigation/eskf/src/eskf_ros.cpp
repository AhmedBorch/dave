#include "eskf/eskf_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "eskf/typedefs.hpp"

auto start_message{R"(
     ________   ______   ___  ____   ________
    |_   __  |.' ____ \ |_  ||_  _| |_   __  |
      | |_ \_|| (___ \_|  | |_/ /     | |_ \_|
      |  _| _  _.____`.   |  __'.     |  _|
     _| |__/ || \____) | _| |  \ \_  _| |_
    |________| \______.'|____||____||_____|
)"};

ESKFNode::ESKFNode(const rclcpp::NodeOptions& options)
    : Node("eskf_node", options) {
    use_tf_transforms_ = this->declare_parameter<bool>("use_tf_transforms");
    tf_sensors_loaded_ = !use_tf_transforms_;

    frame_prefix_ = this->declare_parameter<std::string>("frame_prefix", "");
    if (!frame_prefix_.empty() && frame_prefix_.back() == '/') {
        frame_prefix_.pop_back();
    }
    RCLCPP_INFO(get_logger(), "frame_prefix set to '%s'", frame_prefix_.c_str());

    publish_tf_ = this->declare_parameter<bool>("publish_tf");
    if (publish_tf_) {
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    publish_pose_ = this->declare_parameter<bool>("publish_pose");
    publish_twist_ = this->declare_parameter<bool>("publish_twist");

    publish_biases_ = this->declare_parameter<bool>("publish_biases", true);

    // Declare these here so they appear in `ros2 param list` from startup,
    // even though they are read in complete_initialization().
    this->declare_parameter<int>("publish_rate_ms");
    this->declare_parameter<std::string>("topics.imu");
    this->declare_parameter<std::string>("topics.dvl_twist");
    this->declare_parameter<std::string>("topics.pressure_sensor");
    this->declare_parameter<std::string>("topics.odom");
    this->declare_parameter<std::string>("topics.magnetometer", "");
    this->declare_parameter<std::string>("topics.heading", "");
    this->declare_parameter<std::string>("topics.map_pose", "");
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");

    if (use_tf_transforms_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ESKFNode::lookup_static_transforms, this));
    } else {
        RCLCPP_INFO(get_logger(),
            "Using parameter-based sensor transforms. TF lookup disabled.");
        complete_initialization();
    }
}

void ESKFNode::set_subscribers_and_publisher() {
    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    std::string imu_topic = this->get_parameter("topics.imu").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    std::string dvl_topic = this->get_parameter("topics.dvl_twist").as_string();
    dvl_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos,
        std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));

    std::string mag_topic = this->get_parameter("topics.magnetometer").as_string();
    if (!mag_topic.empty()) {
        mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            mag_topic, qos,
            std::bind(&ESKFNode::mag_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Magnetometer enabled: '%s'", mag_topic.c_str());
    }

    std::string heading_topic = this->get_parameter("topics.heading").as_string();
    if (!heading_topic.empty()) {
        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            heading_topic, qos,
            std::bind(&ESKFNode::heading_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Heading update enabled: '%s'", heading_topic.c_str());
    }

    std::string map_pose_topic = this->get_parameter("topics.map_pose").as_string();
    if (!map_pose_topic.empty()) {
        // Map-matcher publishes RELIABLE; use a reliable subscriber to match.
        auto reliable_qos = rclcpp::QoS(10);
        map_pose_sub_ = this->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            map_pose_topic, reliable_qos,
            std::bind(&ESKFNode::map_pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(),
            "Map-pose update enabled: '%s'", map_pose_topic.c_str());
    }

    std::string pressure_topic = this->get_parameter("topics.pressure_sensor").as_string();
    depth_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            pressure_topic, qos, 
            std::bind(&ESKFNode::depth_callback, this, std::placeholders::_1));

    std::string odom_topic = this->get_parameter("topics.odom").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, qos);

    if (publish_pose_) {
        std::string pose_topic = this->get_parameter("topics.pose").as_string();
        pose_pub_ = this->create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, qos);
    }

    if (publish_twist_) {
        std::string twist_topic = this->get_parameter("topics.twist").as_string();
        twist_pub_ = this->create_publisher<
            geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic, qos);
    }

    if (publish_biases_) {
        accel_bias_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "eskf/accel_bias", qos);
        gyro_bias_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "eskf/gyro_bias", qos);
    }

    accel_aligned_pub_ =
        this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "eskf/accel_aligned", qos);
    gyro_aligned_pub_ =
        this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "eskf/gyro_aligned", qos);

#ifndef NDEBUG
    nis_dvl_pub_ = create_publisher<std_msgs::msg::Float64>(
        "eskf/nis_dvl", rclcpp::QoS(10).reliable());
    nis_depth_pub_ = create_publisher<std_msgs::msg::Float64>(
        "eskf/nis_depth", rclcpp::QoS(10).reliable());
#endif
}

void ESKFNode::set_parameters() {
    if (!use_tf_transforms_) {
        std::vector<double> R_imu_correction =
            this->declare_parameter<std::vector<double>>(
                "transform.imu_frame_r");
        R_imu_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_imu_correction.data());

        std::vector<double> T_imu_correction =
            this->declare_parameter<std::vector<double>>(
                "transform.imu_frame_t");
        T_imu_eskf_ = Eigen::Map<Eigen::Vector3d>(T_imu_correction.data());

        std::vector<double> R_dvl_correction =
            this->declare_parameter<std::vector<double>>(
                "transform.dvl_frame_r");
        R_dvl_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_dvl_correction.data());

        std::vector<double> T_dvl_correction =
            this->declare_parameter<std::vector<double>>(
                "transform.dvl_frame_t");
        T_dvl_eskf_ = Eigen::Map<Eigen::Vector3d>(T_dvl_correction.data());

        // std::vector<double> T_depth_correction =
        //     this->declare_parameter<std::vector<double>>(
        //         "transform.depth_frame_t");
        // T_depth_eskf_ = Eigen::Map<Eigen::Vector3d>(T_depth_correction.data());
    }

    std::vector<double> diag_Q_std;
    this->declare_parameter<std::vector<double>>("diag_Q_std");

    diag_Q_std = this->get_parameter("diag_Q_std").as_double_array();

    if (diag_Q_std.size() != 12) {
        throw std::runtime_error("diag_Q_std must have length 12");
    }

    Eigen::Matrix12d Q = Eigen::Map<const Eigen::Vector12d>(diag_Q_std.data())
                             .array()
                             .square()
                             .matrix()
                             .asDiagonal();

    std::vector<double> diag_p_init =
        this->declare_parameter<std::vector<double>>("diag_p_init");
    if (diag_p_init.size() != 15) {
        throw std::runtime_error("diag_p_init must have length 15");
    }
    Eigen::Matrix15d P = createDiagonalMatrix<15>(diag_p_init);

    Eigen::Vector3d g_vec(0.0, 0.0, this->gravity);

    std::vector<double> initial_gyro_bias =
        this->declare_parameter<std::vector<double>>(
            "initial_gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
    RCLCPP_INFO(get_logger(), "initial_gyro_bias: [%f, %f, %f]",
                initial_gyro_bias[0], initial_gyro_bias[1], initial_gyro_bias[2]);
    if (initial_gyro_bias.size() != 3) {
        throw std::runtime_error("initial_gyro_bias must have length 3");
    }

    std::vector<double> initial_accel_bias =
        this->declare_parameter<std::vector<double>>(
            "initial_accel_bias", std::vector<double>{0.0, 0.0, 0.0});
    RCLCPP_INFO(get_logger(), "initial_accel_bias: [%f, %f, %f]",
                initial_accel_bias[0], initial_accel_bias[1], initial_accel_bias[2]);
    if (initial_accel_bias.size() != 3) {
        throw std::runtime_error("initial_accel_bias must have length 3");
    }

    EskfParams eskf_params{
        .Q = Q,
        .P = P,
        .g_ = g_vec,
        .initial_gyro_bias =
            Eigen::Map<Eigen::Vector3d>(initial_gyro_bias.data()),
        .initial_accel_bias =
            Eigen::Map<Eigen::Vector3d>(initial_accel_bias.data())};

    eskf_ = std::make_unique<ESKF>(eskf_params);

    std::vector<double> mag_ref =
        this->declare_parameter<std::vector<double>>(
            "mag_reference_field", std::vector<double>{0.0, 1.73e-5, -5.32e-5});
    mag_reference_field_ = Eigen::Map<Eigen::Vector3d>(mag_ref.data());
    double mag_noise_std =
        this->declare_parameter<double>("mag_noise_std", 2e-4);
    mag_noise_ = Eigen::Matrix3d::Identity() * (mag_noise_std * mag_noise_std);
    RCLCPP_INFO(get_logger(),
        "Mag reference field: [%.3e, %.3e, %.3e] T",
        mag_reference_field_.x(), mag_reference_field_.y(), mag_reference_field_.z());

    add_gravity_to_imu_ = this->declare_parameter<bool>("add_gravity_to_imu");
    RCLCPP_INFO(get_logger(), "add_gravity_to_imu: %s",
                add_gravity_to_imu_ ? "true" : "false");

    heading_noise_var_ =
        std::pow(this->declare_parameter<double>("heading_noise_std", 0.05), 2);
}

void ESKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time current_time = msg->header.stamp;

    if (!first_imu_msg_received_) {
        last_imu_time_ = current_time;
        first_imu_msg_received_ = true;
        return;
    }

    double dt = (current_time - last_imu_time_).nanoseconds() * 1e-9;
    last_imu_time_ = current_time;

    ImuMeasurement imu_measurement{};

    Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);

    Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);

    Eigen::Vector3d accel_aligned = R_imu_eskf_ * raw_accel;

    geometry_msgs::msg::Vector3Stamped accel_aligned_msg;
    accel_aligned_msg.header.stamp = current_time;
    accel_aligned_msg.header.frame_id = frame("base_link");
    accel_aligned_msg.vector.x = accel_aligned.x();
    accel_aligned_msg.vector.y = accel_aligned.y();
    accel_aligned_msg.vector.z = accel_aligned.z();
    accel_aligned_pub_->publish(accel_aligned_msg);

    // currently the gyro and the accelorometer are rotated differently in sim.
    // should be changed with the actual drone params.
    Eigen::Vector3d gyro_aligned = R_imu_eskf_ * raw_gyro;
    imu_measurement.gyro = gyro_aligned;

    geometry_msgs::msg::Vector3Stamped gyro_aligned_msg;
    gyro_aligned_msg.header.stamp = current_time;
    gyro_aligned_msg.header.frame_id = frame("base_link");
    gyro_aligned_msg.vector.x = gyro_aligned.x();
    gyro_aligned_msg.vector.y = gyro_aligned.y();
    gyro_aligned_msg.vector.z = gyro_aligned.z();
    gyro_aligned_pub_->publish(gyro_aligned_msg);

    // lever arm correction for accelerometer
    StateQuat nom_state = eskf_->get_nominal_state();
    Eigen::Vector3d omega = gyro_aligned - nom_state.gyro_bias;

    // a_corrected = a_meas - omega x (omega x T)
    Eigen::Vector3d centripetal_accel = omega.cross(omega.cross(T_imu_eskf_));
    accel_aligned -= centripetal_accel;

    if (add_gravity_to_imu_) {
        Eigen::Matrix3d R = nom_state.quat.normalized().toRotationMatrix();
        accel_aligned -= R.transpose() * eskf_->get_gravity();
    }

    imu_measurement.accel = accel_aligned;

    // save latest gyro readings (used for DVL correction and odom output)
    latest_gyro_measurement_ = imu_measurement.gyro;

    eskf_->imu_update(imu_measurement, dt);
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    SensorDVL dvl_sensor;

    dvl_sensor.measurement << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y, msg->twist.twist.linear.z;

    // Extract the 3x3 linear velocity block from the 6x6 row-major covariance.
    // In a TwistWithCovariance the layout is [vx,vy,vz,wx,wy,wz], so the
    // linear sub-block occupies indices [0,1,2 | 6,7,8 | 12,13,14].
    const auto& c = msg->twist.covariance;
    dvl_sensor.measurement_noise <<
        c[0],  c[1],  c[2],
        c[6],  c[7],  c[8],
        c[12], c[13], c[14];

    // Apply the rotation and translation corrections to the DVL measurement
    StateQuat nom_state = eskf_->get_nominal_state();
    // get the angular velocity
    Eigen::Vector3d omega_corrected =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    // correct rotation and translation: v_base = v_sensor - omega x T
    dvl_sensor.measurement = R_dvl_eskf_ * dvl_sensor.measurement -
                             omega_corrected.cross(T_dvl_eskf_);
    dvl_sensor.measurement_noise =
        R_dvl_eskf_ * dvl_sensor.measurement_noise * R_dvl_eskf_.transpose();

    eskf_->dvl_update(dvl_sensor);

#ifndef NDEBUG
    // Publish NIS in Debug mode
    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->get_nis();
    nis_dvl_pub_->publish(nis_msg);
#endif
}

void ESKFNode::mag_callback(
    const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    if (!first_imu_msg_received_) return;
    // gz-sim bridge outputs in Gauss with body-NED axes (x=North, y=East, z=Down).
    // Convert to body-FLU Tesla (x=East, y=North, z=Up) that the ESKF expects.
    constexpr double GAUSS_TO_TESLA = 1e-4;
    const double g_N = msg->magnetic_field.x;
    const double g_E = msg->magnetic_field.y;
    const double g_D = msg->magnetic_field.z;
    SensorMag mag_sensor;
    mag_sensor.measurement << g_E * GAUSS_TO_TESLA,
                               g_N * GAUSS_TO_TESLA,
                              -g_D * GAUSS_TO_TESLA;
    mag_sensor.reference_field   = mag_reference_field_;
    mag_sensor.measurement_noise = mag_noise_;
    eskf_->mag_update(mag_sensor);
}

void ESKFNode::heading_callback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    if (!first_imu_msg_received_) return;

    SensorYaw yaw_sensor;
    yaw_sensor.measurement = msg->data;
    // Noise variance is declared by the heading_publisher node.
    // Use a fixed small value here — the publisher already added the noise.
    yaw_sensor.measurement_noise = heading_noise_var_;  // set via heading_noise_std param
    eskf_->yaw_update(yaw_sensor);
}

void ESKFNode::map_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!first_imu_msg_received_) return;

    SensorPose pose_sensor;

    const auto& p = msg->pose.pose.position;
    pose_sensor.position = Eigen::Vector3d(p.x, p.y, p.z);

    const auto& q = msg->pose.pose.orientation;
    pose_sensor.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();

    // PoseWithCovariance stores covariance row-major [x,y,z,rot_x,rot_y,rot_z]^2
    pose_sensor.covariance =
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
            msg->pose.covariance.data());

    eskf_->pose_update(pose_sensor);
}

void ESKFNode::depth_callback(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    SensorDepth depth_sensor;
    // Plugin publishes absolute pressure in kPa: P = P_atm + |z| * kPa_per_meter.
    // Invert to z in ENU (negative underwater).
    depth_sensor.measurement =
        -(msg->fluid_pressure - depth_standard_pressure_kPa_) / depth_kPa_per_meter_;
    // msg->variance is in kPa²; convert to m²
    depth_sensor.measurement_noise =
        msg->variance / (depth_kPa_per_meter_ * depth_kPa_per_meter_);
    eskf_->depth_update(depth_sensor);

#ifndef NDEBUG
    // Publish NIS in Debug mode
    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->get_nis();
    nis_depth_pub_->publish(nis_msg);
#endif
}

void ESKFNode::publish_odom() {
    nav_msgs::msg::Odometry odom_msg;
    StateQuat nom_state = eskf_->get_nominal_state();
    StateEuler error_state_ = eskf_->get_error_state();

    odom_msg.pose.pose.position.x = nom_state.pos.x();
    odom_msg.pose.pose.position.y = nom_state.pos.y();
    odom_msg.pose.pose.position.z = nom_state.pos.z();

    odom_msg.pose.pose.orientation.w = nom_state.quat.w();
    odom_msg.pose.pose.orientation.x = nom_state.quat.x();
    odom_msg.pose.pose.orientation.y = nom_state.quat.y();
    odom_msg.pose.pose.orientation.z = nom_state.quat.z();

    // publishing the velocity in the body frame
    Eigen::Matrix3d R_body_to_world = nom_state.quat.toRotationMatrix();

    Eigen::Vector3d v_body = R_body_to_world.transpose() * nom_state.vel;

    odom_msg.twist.twist.linear.x = v_body.x();
    odom_msg.twist.twist.linear.y = v_body.y();
    odom_msg.twist.twist.linear.z = v_body.z();

    // Add bias values to the angular velocity field of twist
    Eigen::Vector3d body_angular_vel =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    odom_msg.twist.twist.angular.x = body_angular_vel.x();
    odom_msg.twist.twist.angular.y = body_angular_vel.y();
    odom_msg.twist.twist.angular.z = body_angular_vel.z();

    // If you also want to include gyro bias, you could add it to the covariance
    // matrix or publish a separate topic for biases
    rclcpp::Time current_time = this->now();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame("odom");

    // Some cross terms of the covariance are ignored, and the acc/gyro biases
    // cov are not published. Pos and orientation cov needs to be mapped from
    // 6*6 matrix to an array (states 0-2)

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            odom_msg.pose.covariance[i * 6 + j] = error_state_.covariance(i, j);
        }
    }

    // Orientation covariance (states 6–8)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            odom_msg.pose.covariance[(i + 3) * 6 + (j + 3)] =
                error_state_.covariance(i + 6, j + 6);
        }
    }

    // Linear velocity covariance
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            odom_msg.twist.covariance[i * 6 + j] =
                error_state_.covariance(i + 3, j + 3);
        }
    }
    odom_pub_->publish(odom_msg);

    if (publish_pose_) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose;
        pose_pub_->publish(pose_msg);
    }

    if (publish_twist_) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = odom_msg.header;
        twist_msg.twist = odom_msg.twist;
        twist_pub_->publish(twist_msg);
    }

    if (publish_tf_) {
        publish_tf(nom_state, current_time);
    }

    if (publish_biases_) {
        geometry_msgs::msg::Vector3Stamped accel_bias_msg;
        accel_bias_msg.header.stamp = current_time;
        accel_bias_msg.header.frame_id =
            frame("base_link");  // Biases are in the body frame

        accel_bias_msg.vector.x = nom_state.accel_bias.x();
        accel_bias_msg.vector.y = nom_state.accel_bias.y();
        accel_bias_msg.vector.z = nom_state.accel_bias.z();

        accel_bias_pub_->publish(accel_bias_msg);

        geometry_msgs::msg::Vector3Stamped gyro_bias_msg;
        gyro_bias_msg.header = accel_bias_msg.header;

        gyro_bias_msg.vector.x = nom_state.gyro_bias.x();
        gyro_bias_msg.vector.y = nom_state.gyro_bias.y();
        gyro_bias_msg.vector.z = nom_state.gyro_bias.z();

        gyro_bias_pub_->publish(gyro_bias_msg);
    }
}

void ESKFNode::lookup_static_transforms() {
    try {
        Tf_base_imu_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame("imu_link"), tf2::TimePointZero));
        R_imu_eskf_ = Tf_base_imu_.rotation();
        T_imu_eskf_ = Tf_base_imu_.translation();

        Tf_base_dvl_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame("dvl_link"), tf2::TimePointZero));
        R_dvl_eskf_ = Tf_base_dvl_.rotation();
        T_dvl_eskf_ = Tf_base_dvl_.translation();

        Tf_base_depth_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame("pressure_sensor_link"),
            tf2::TimePointZero));
        T_depth_eskf_ = Tf_base_depth_.translation();

        tf_sensors_loaded_ = true;
        tf_timer_->cancel();
        RCLCPP_INFO(get_logger(), "All static transforms loaded successfully.");
        complete_initialization();
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF Lookup failed (will retry): %s", ex.what());
    }
}

void ESKFNode::complete_initialization() {
    set_subscribers_and_publisher();
    this->gravity = -this->declare_parameter<double>("gravity", 9.8);
    // Depth conversion uses the same constants as the sea_pressure_sensor SDF plugin.
    // Plugin publishes total pressure in kPa: P = standard_pressure + |z| * kPa_per_meter.
    this->depth_standard_pressure_kPa_ =
        this->declare_parameter<double>("depth_standard_pressure_kPa", 101.325);
    this->depth_kPa_per_meter_ =
        this->declare_parameter<double>("depth_kPa_per_meter", 9.80638);
    set_parameters();

    time_step_ = std::chrono::milliseconds(
        this->get_parameter("publish_rate_ms").as_int());
    odom_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&ESKFNode::publish_odom, this));

    RCLCPP_INFO(get_logger(), "%s", start_message);

#ifndef NDEBUG
    RCLCPP_INFO(get_logger(),
        "______________________Debug mode is enabled______________________");
#endif
}

void ESKFNode::publish_tf(const StateQuat& nom_state,
                          const rclcpp::Time& time) {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = frame("odom");
    tf_msg.child_frame_id = frame("base_link");

    tf_msg.transform.translation.x = nom_state.pos.x();
    tf_msg.transform.translation.y = nom_state.pos.y();
    tf_msg.transform.translation.z = nom_state.pos.z();

    tf_msg.transform.rotation.w = nom_state.quat.w();
    tf_msg.transform.rotation.x = nom_state.quat.x();
    tf_msg.transform.rotation.y = nom_state.quat.y();
    tf_msg.transform.rotation.z = nom_state.quat.z();

    tf_broadcaster_->sendTransform(tf_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ESKFNode)
