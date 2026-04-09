
#include "legged_ros_hw/RosHW.h"
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <algorithm>
#include <cctype>

namespace legged
{
  namespace
  {
    bool containsIgnoreCase(const std::string &text, const std::string &pattern)
    {
      std::string loweredText = text;
      std::string loweredPattern = pattern;
      std::transform(loweredText.begin(), loweredText.end(), loweredText.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      std::transform(loweredPattern.begin(), loweredPattern.end(), loweredPattern.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      return loweredText.find(loweredPattern) != std::string::npos;
    }
  } // namespace

  bool RosHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {
    if (!LeggedHW::init(root_nh, robot_hw_nh))
    {
      return false;
    }

    // Get topic names from parameter server
    robot_hw_nh.param<std::string>("joint_state_topic", joint_state_topic_, "/state/joint");
    robot_hw_nh.param<std::string>("imu_topic", imu_topic_, "/state/imu");
    robot_hw_nh.param<std::string>("contact_state_topic", contact_state_topic_, "/state/contact");
    robot_hw_nh.param<std::string>("joint_cmd_topic", joint_cmd_topic_, "/desire/joint");
    robot_hw_nh.param<std::string>("joint_params_topic", joint_params_topic_, "/desire/joint/params");

    if (!setupJoints())
    {
      ROS_ERROR("Failed to setup joints");
      return false;
    }
    if (!setupImu())
    {
      ROS_ERROR("Failed to setup IMU");
      return false;
    }
    if (!setupContacts())
    {
      ROS_ERROR("Failed to setup contacts");
      return false;
    }

    joint_names_ = hybridJointInterface_.getNames();
    const size_t num_joints = joint_names_.size();

    js_msg_.name.reserve(num_joints);
    js_msg_.position.reserve(num_joints);
    js_msg_.velocity.reserve(num_joints);
    js_msg_.effort.reserve(num_joints);

    for (const auto &name : joint_names_)
    {
      js_msg_.name.push_back(name);
      js_msg_.position.push_back(0.0);
      js_msg_.velocity.push_back(0.0);
      js_msg_.effort.push_back(0.0);
    }

    joint_cmd_msg_.name = joint_names_;
    joint_cmd_msg_.position.assign(num_joints, 0.0);
    joint_cmd_msg_.velocity.assign(num_joints, 0.0);
    joint_cmd_msg_.effort.assign(num_joints, 0.0);

    joint_params_msg_.data.assign(num_joints * 2, 0.0);
    joint_params_msg_.layout.dim.resize(2);
    joint_params_msg_.layout.dim[0].label = "joints";
    joint_params_msg_.layout.dim[0].size = num_joints;
    joint_params_msg_.layout.dim[0].stride = 2;
    joint_params_msg_.layout.dim[1].label = "params";
    joint_params_msg_.layout.dim[1].size = 2;
    joint_params_msg_.layout.dim[1].stride = 1;
    joint_params_msg_.layout.data_offset = 0;

    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
    imu_msg_.angular_velocity.x = 0.0;
    imu_msg_.angular_velocity.y = 0.0;
    imu_msg_.angular_velocity.z = 0.0;
    imu_msg_.linear_acceleration.x = 0.0;
    imu_msg_.linear_acceleration.y = 0.0;
    imu_msg_.linear_acceleration.z = 0.0;

    for (size_t i = 0; i < 9; ++i)
    {
      imu_msg_.orientation_covariance[i] = 0.0;
      imu_msg_.angular_velocity_covariance[i] = 0.0;
      imu_msg_.linear_acceleration_covariance[i] = 0.0;
    }

    joint_cmd_pub_ = root_nh.advertise<sensor_msgs::JointState>(joint_cmd_topic_, 1);
    joint_params_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>(joint_params_topic_, 1);

    joint_state_sub_ = root_nh.subscribe<sensor_msgs::JointState>(joint_state_topic_, 1, &RosHW::jointStateCallback, this);
    imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>(imu_topic_, 1, &RosHW::imuCallback, this);
    contact_state_sub_ = root_nh.subscribe<std_msgs::Float64MultiArray>(contact_state_topic_, 1, &RosHW::contactStateCallback, this);

    return true;
  }

  void RosHW::read(const ros::Time &time, const ros::Duration & /*period*/)
  {
    {
      std::lock_guard<std::mutex> guard(mtx_js_);
      const size_t nj = std::min(
          joint_data_.size(),
          std::min(js_msg_.position.size(), std::min(js_msg_.velocity.size(), js_msg_.effort.size())));
      for (size_t i = 0; i < nj; ++i)
      {
        joint_data_[i].pos_ = js_msg_.position[i];
        joint_data_[i].vel_ = js_msg_.velocity[i];
        joint_data_[i].tau_ = js_msg_.effort[i];
      }
    }

    {
      std::lock_guard<std::mutex> guard(mtx_imu_);
      imu_data_.ori_[0] = imu_msg_.orientation.x;
      imu_data_.ori_[1] = imu_msg_.orientation.y;
      imu_data_.ori_[2] = imu_msg_.orientation.z;
      imu_data_.ori_[3] = imu_msg_.orientation.w;

      imu_data_.angular_vel_[0] = imu_msg_.angular_velocity.x;
      imu_data_.angular_vel_[1] = imu_msg_.angular_velocity.y;
      imu_data_.angular_vel_[2] = imu_msg_.angular_velocity.z;

      imu_data_.linear_acc_[0] = imu_msg_.linear_acceleration.x;
      imu_data_.linear_acc_[1] = imu_msg_.linear_acceleration.y;
      imu_data_.linear_acc_[2] = imu_msg_.linear_acceleration.z;

      for (size_t i = 0; i < 9; ++i)
      {
        imu_data_.ori_cov_[i] = imu_msg_.orientation_covariance[i];
        imu_data_.angular_vel_cov_[i] = imu_msg_.angular_velocity_covariance[i];
        imu_data_.linear_acc_cov_[i] = imu_msg_.linear_acceleration_covariance[i];
      }
    }

    std::lock_guard<std::mutex> guard(mtx_contact_);
    for (size_t i = 0; i < contact_states_.size() && i < contact_state_msg_.data.size(); ++i)
    {
      contact_states_[i] = (contact_state_msg_.data[i] > 0.5) ? 1 : 0;
    }
  }

  void RosHW::write(const ros::Time & /*time*/, const ros::Duration & /*period*/)
  {
    joint_cmd_msg_.header.stamp = ros::Time::now();
    const size_t num_joints = joint_names_.size();

    for (size_t i = 0; i < num_joints; ++i)
    {
      try
      {
        HybridJointHandle handle = hybridJointInterface_.getHandle(joint_names_[i]);

        joint_cmd_msg_.position[i] = handle.getPositionDesired();
        joint_cmd_msg_.velocity[i] = handle.getVelocityDesired();
        joint_cmd_msg_.effort[i] = handle.getFeedforward();

        // [joint0_kp, joint0_kd, joint1_kp, joint1_kd, ...]
        const size_t base = i * 2;
        joint_params_msg_.data[base] = handle.getKp();
        joint_params_msg_.data[base + 1] = handle.getKd();
      }
      catch (const std::exception &e)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "Failed to get handle for joint " << joint_names_[i] << ": " << e.what());
      }
    }

    joint_cmd_pub_.publish(joint_cmd_msg_);
    joint_params_pub_.publish(joint_params_msg_);
  }

  bool RosHW::setupJoints()
  {
    joint_data_.clear();
    joint_name_to_index_.clear();

    if (urdfModel_ == nullptr)
    {
      ROS_ERROR("URDF model is not initialized");
      return false;
    }

    uint16_t nj = urdfModel_->joints_.size();
    joint_data_.reserve(nj);

    size_t index = 0;
    for (const auto &joint : urdfModel_->joints_)
    {
      if (joint.second->type != urdf::Joint::REVOLUTE &&
          joint.second->type != urdf::Joint::CONTINUOUS)
      {
        continue;
      }

      // Create joint data
      RosJointData rj_data;
      rj_data.pos_des_ = 0;
      rj_data.vel_des_ = 0;
      rj_data.ff_ = 0;
      rj_data.kp_ = 0;
      rj_data.kd_ = 0;
      joint_data_.push_back(rj_data);
      joint_name_to_index_[joint.first] = index;

      // Register joint state interface
      hardware_interface::JointStateHandle state_handle(
          joint.first,
          &joint_data_[index].pos_,
          &joint_data_[index].vel_,
          &joint_data_[index].tau_);
      jointStateInterface_.registerHandle(state_handle);

      // Register hybrid joint interface
      hybridJointInterface_.registerHandle(HybridJointHandle(
          state_handle,
          &joint_data_[index].pos_des_,
          &joint_data_[index].vel_des_,
          &joint_data_[index].kp_,
          &joint_data_[index].kd_,
          &joint_data_[index].ff_));
      ROS_INFO_STREAM("Registered joint: " << joint.first);

      index++;
    }
    ROS_INFO_STREAM("Initialized " << index << " joint");

    return true;
  }

  bool RosHW::setupImu()
  {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
        "base_imu", "base_imu",
        imu_data_.ori_, imu_data_.ori_cov_,
        imu_data_.angular_vel_, imu_data_.angular_vel_cov_,
        imu_data_.linear_acc_, imu_data_.linear_acc_cov_));
    ROS_INFO_STREAM("Registered imu: " << "base_imu");

    // Set default covariance values
    imu_data_.ori_cov_[0] = 0.0012;
    imu_data_.ori_cov_[4] = 0.0012;
    imu_data_.ori_cov_[8] = 0.0012;

    imu_data_.angular_vel_cov_[0] = 0.0004;
    imu_data_.angular_vel_cov_[4] = 0.0004;
    imu_data_.angular_vel_cov_[8] = 0.0004;

    imu_data_.linear_acc_cov_[0] = 0.01;
    imu_data_.linear_acc_cov_[4] = 0.01;
    imu_data_.linear_acc_cov_[8] = 0.01;

    return true;
  }

  bool RosHW::setupContacts()
  {
    contact_names_.clear();
    contact_states_.clear();

    if (urdfModel_ == nullptr)
    {
      ROS_ERROR("URDF model is not initialized");
      return false;
    }

    for (const auto &link : urdfModel_->links_)
    {
      if (containsIgnoreCase(link.first, "contact"))
      {
        contact_names_.push_back(link.first);
      }
    }

    if (contact_names_.empty())
    {
      for (const auto &link : urdfModel_->links_)
      {
        if (containsIgnoreCase(link.first, "foot"))
        {
          contact_names_.push_back(link.first);
        }
      }
      if (!contact_names_.empty())
      {
        ROS_INFO_STREAM("No link name contains 'contact'. Falling back to link names containing 'foot'.");
      }
    }
    else
    {
      ROS_INFO_STREAM("Found link names containing 'contact'. Only these links are used as contact sensors.");
    }

    if (contact_names_.empty())
    {
      ROS_WARN("No contact links found in URDF (neither 'contact' nor 'foot' in link names).");
      return true;
    }

    const uint16_t nc = contact_names_.size();
    contact_states_.reserve(nc);
    contact_state_msg_.data.assign(nc, 0.0);

    for (const auto &name : contact_names_)
    {
      contact_states_.push_back(0);

      contactSensorInterface_.registerHandle(
          ContactSensorHandle(name, reinterpret_cast<const bool *>(&contact_states_.back())));

      ROS_INFO_STREAM("Registered contact sensor: " << name);
    }

    ROS_INFO_STREAM("Initialized " << contact_names_.size() << " contact sensors");
    return true;
  }

  void RosHW::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
  {
    std::lock_guard<std::mutex> guard(mtx_js_);
    const size_t nj = js_msg_.name.size();
    for (size_t i = 0; i < nj; ++i)
    {
      if (i < msg->position.size())
      {
        js_msg_.position[i] = msg->position[i];
      }
      if (i < msg->velocity.size())
      {
        js_msg_.velocity[i] = msg->velocity[i];
      }
      if (i < msg->effort.size())
      {
        js_msg_.effort[i] = msg->effort[i];
      }
    }
  }

  void RosHW::imuCallback(const sensor_msgs::ImuConstPtr &msg)
  {
    std::lock_guard<std::mutex> guard(mtx_imu_);
    imu_msg_ = *msg;
  }

  void RosHW::contactStateCallback(const std_msgs::Float64MultiArrayConstPtr &msg)
  {
    std::lock_guard<std::mutex> guard(mtx_contact_);
    const size_t nc = contact_state_msg_.data.size();
    for (size_t i = 0; i < nc && i < msg->data.size(); ++i)
    {
      contact_state_msg_.data[i] = msg->data[i];
    }
  }

} // namespace legged
