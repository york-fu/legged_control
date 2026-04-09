
#pragma once

#include <legged_hw/LeggedHW.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <vector>
#include <map>
#include <string>

namespace legged
{

  struct RosJointData
  {
    double pos_, vel_, tau_;                  // state
    double pos_des_, vel_des_, kp_, kd_, ff_; // command
  };

  struct RosImuData
  {
    double ori_[4];             // NOLINT(modernize-avoid-c-arrays)
    double ori_cov_[9];         // NOLINT(modernize-avoid-c-arrays)
    double angular_vel_[3];     // NOLINT(modernize-avoid-c-arrays)
    double angular_vel_cov_[9]; // NOLINT(modernize-avoid-c-arrays)
    double linear_acc_[3];      // NOLINT(modernize-avoid-c-arrays)
    double linear_acc_cov_[9];  // NOLINT(modernize-avoid-c-arrays)
  };

  class RosHW : public LeggedHW
  {
  public:
    RosHW() = default;
    /** \brief Get necessary params from param server. Init hardware_interface.
     *
     * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
     * joint limit. Initialize ROS subscribers and publishers.
     *
     * @param root_nh Root node-handle of a ROS node.
     * @param robot_hw_nh Node-handle for robot hardware.
     * @return True when init successful, False when failed.
     */
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
    /** \brief Communicate with hardware. Get data, status of robot.
     *
     * Read data from subscribed ROS topics and update joint and IMU states.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /** \brief Communicate with hardware. Publish command to robot.
     *
     * Publish joint commands via ROS topics.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

  private:
    bool setupJoints();
    bool setupImu();
    bool setupContacts();

    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void contactStateCallback(const std_msgs::Float64MultiArrayConstPtr &msg);

    std::vector<RosJointData> joint_data_;
    RosImuData imu_data_{};

    sensor_msgs::JointState js_msg_;
    sensor_msgs::Imu imu_msg_;
    std_msgs::Float64MultiArray contact_state_msg_;

    std::mutex mtx_js_;
    std::mutex mtx_imu_;
    std::mutex mtx_contact_;

    ros::Subscriber joint_state_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber contact_state_sub_;

    ros::Publisher joint_cmd_pub_;
    ros::Publisher joint_params_pub_;
    std::string joint_state_topic_;
    std::string imu_topic_;
    std::string contact_state_topic_;
    std::string joint_cmd_topic_;
    std::string joint_params_topic_;

    std::map<std::string, size_t> joint_name_to_index_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> contact_names_;
    std::vector<char> contact_states_;

    sensor_msgs::JointState joint_cmd_msg_;
    std_msgs::Float64MultiArray joint_params_msg_;
  };

} // namespace legged
