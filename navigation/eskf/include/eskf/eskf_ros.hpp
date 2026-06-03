#ifndef ESKF__ESKF_ROS_HPP_
#define ESKF__ESKF_ROS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include "eskf/eskf.hpp"
#include "eskf/typedefs.hpp"

class ESKFNode : public rclcpp::Node {
   public:
    explicit ESKFNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    // @brief Callback function for the imu topic
    // @param msg: Imu message containing the imu data
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // @brief Callback function for the dvl topic
    // @param msg: DVL message containing the dvl data
    void dvl_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    void depth_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

    // @brief Inject a noisy ground-truth yaw as a 1-DOF heading correction.
    void gt_yaw_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // @brief Update ESKF with a 6-DOF pose from the map-matcher.
    void map_pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // @brief Publish the odometry message
    void publish_odom();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

    // @brief lookup transforms
    void lookup_static_transforms();

    // @brief Create subs/pubs and start the publish timer. Called once
    // transforms are available (or immediately if use_tf_transforms_ is false).
    void complete_initialization();

    // @brief broadcast the State as a TF
    void publish_tf(const StateQuat& nom_state,
                    const rclcpp::Time& current_time);

    // Subscribers and Publishers

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr depth_sub_;

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_yaw_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_pose_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_dvl_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_depth_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        accel_bias_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        gyro_bias_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        accel_aligned_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        gyro_aligned_pub_;

    // Member variable for the ESKF instance

    std::chrono::milliseconds time_step_{1};

    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    std::unique_ptr<ESKF> eskf_;

    bool first_imu_msg_received_ = false;

    Eigen::Matrix3d R_imu_eskf_{};
    Eigen::Vector3d T_imu_eskf_{};

    Eigen::Matrix3d R_dvl_eskf_{};
    Eigen::Vector3d T_dvl_eskf_{};

    Eigen::Vector3d T_depth_eskf_{};

    Eigen::Vector3d mag_reference_field_{};  // world-frame B-field (T)

    // GT yaw noise injection (placeholder until the real magnetometer pathway
    // is sorted). Subscribes to a Gazebo odometry topic and feeds yaw + N(0,σ²)
    // into the ESKF as a 1-DOF heading measurement.
    double yaw_gt_noise_std_{0.4};  // σ in radians (~3°) for 0.05
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<double> yaw_noise_dist_{0.0, 1.0};
    Eigen::Matrix3d mag_noise_{};            // measurement noise covariance (T²)

    rclcpp::Time last_imu_time_{};

    // Latest gyro measurement (used for publishing odom output of eskf)
    Eigen::Vector3d latest_gyro_measurement_{};

    // TF2 Handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    std::string frame(const std::string& name) const {
        return frame_prefix_.empty() ? name : frame_prefix_ + "/" + name;
    }

    // Flags and Storage
    std::string frame_prefix_{""};
    bool use_tf_transforms_ = false;
    bool tf_sensors_loaded_ = false;
    bool publish_tf_{false};
    bool publish_pose_{false};
    bool publish_twist_{false};
    bool publish_biases_{false};
    bool add_gravity_to_imu_{false};

    // hold the transfer from Sensor -> Base Link
    Eigen::Isometry3d Tf_base_imu_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tf_base_dvl_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tf_base_depth_ = Eigen::Isometry3d::Identity();

    double gravity;
    double depth_standard_pressure_kPa_;  // must match <standard_pressure> in sensor SDF
    double depth_kPa_per_meter_;          // must match <kPa_per_meter> in sensor SDF
};

#endif  // ESKF__ESKF_ROS_HPP_
