/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_hw_api/api.h>

#include <nav_msgs/msg/odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/node.h>

#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_lib/gps_conversions.h>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_coppelia_api
{

/* class Api //{ */

class Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  Api() = default;
  ~Api() = default;

  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) override;
  void destroy() override;

  // | ------------------------- params ------------------------- |

  mrs_msgs::msg::HwApiCapabilities _capabilities_;

  std::string _uav_name_;
  std::string _world_frame_name_;
  std::string _body_frame_name_;

  double      _utm_x_;
  double      _utm_y_;
  std::string _utm_zone_;
  double      _amsl_;

  // | --------------------- status methods --------------------- |

  mrs_msgs::msg::HwApiStatus       getStatus();
  mrs_msgs::msg::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool& request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- parameters ----------------------- |

  std::string _simulator_prefix_;

  std::string _topic_simulator_odom_;
  std::string _topic_simulator_imu_;
  std::string _topic_simulator_rangefinder_;

  std::string _topic_simulator_actuators_cmd_;
  std::string _topic_simulator_control_group_cmd_;
  std::string _topic_simulator_attitude_rate_cmd_;
  std::string _topic_simulator_attitude_cmd_;
  std::string _topic_simulator_acceleration_hdg_rate_cmd_;
  std::string _topic_simulator_velocity_hdg_rate_cmd_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odom_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>   sh_imu_;

  void callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>            ph_actuators_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiControlGroupCmd>        ph_control_group_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>        ph_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeCmd>            ph_attitude_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd> ph_acceleration_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>     ph_velocity_hdg_rate_cmd_;

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<TimerType> timer_main_;
  void timerMain();

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = true;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;

  // | ------------------------- methods ------------------------ |

  void publishBatteryState(void);

  void publishRC(void);

  void publishDistanceSensor(void);

  void timeoutInputs(void);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Api::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  node_ = node;
  clock_ = node_->get_clock();

  common_handlers_ = common_handlers;

  _uav_name_         = common_handlers->getUavName();
  _body_frame_name_  = common_handlers->getBodyFrameName();
  _world_frame_name_ = common_handlers->getWorldFrameName();

  _capabilities_.api_name = "CoppeliaApi";

  // | ------------------- loading parameters ------------------- |

  auto &param_loader = *common_handlers->main_param_loader;
  param_loader.setPrefix("plugin/");

  param_loader.loadParam("gnss/utm_x", _utm_x_);
  param_loader.loadParam("gnss/utm_y", _utm_y_);
  param_loader.loadParam("gnss/utm_zone", _utm_zone_);
  param_loader.loadParam("gnss/amsl", _amsl_);

  param_loader.loadParam("input_mode/actuators", (bool&)_capabilities_.accepts_actuator_cmd);
  param_loader.loadParam("input_mode/control_group", (bool&)_capabilities_.accepts_control_group_cmd);
  param_loader.loadParam("input_mode/attitude_rate", (bool&)_capabilities_.accepts_attitude_rate_cmd);
  param_loader.loadParam("input_mode/attitude", (bool&)_capabilities_.accepts_attitude_cmd);
  param_loader.loadParam("input_mode/acceleration_hdg_rate", (bool&)_capabilities_.accepts_acceleration_hdg_rate_cmd);
  param_loader.loadParam("input_mode/velocity_hdg_rate", (bool&)_capabilities_.accepts_velocity_hdg_rate_cmd);

  param_loader.loadParam("outputs/ground_truth", (bool&)_capabilities_.produces_ground_truth);
  param_loader.loadParam("outputs/distance_sensor", (bool&)_capabilities_.produces_distance_sensor);
  param_loader.loadParam("outputs/gnss", (bool&)_capabilities_.produces_gnss);
  param_loader.loadParam("outputs/imu", (bool&)_capabilities_.produces_imu);
  param_loader.loadParam("outputs/altitude", (bool&)_capabilities_.produces_altitude);
  param_loader.loadParam("outputs/magnetometer_heading", (bool&)_capabilities_.produces_magnetometer_heading);
  param_loader.loadParam("outputs/rc_channels", (bool&)_capabilities_.produces_rc_channels);
  param_loader.loadParam("outputs/battery_state", (bool&)_capabilities_.produces_battery_state);
  param_loader.loadParam("outputs/position", (bool&)_capabilities_.produces_position);
  param_loader.loadParam("outputs/orientation", (bool&)_capabilities_.produces_orientation);
  param_loader.loadParam("outputs/velocity", (bool&)_capabilities_.produces_velocity);
  param_loader.loadParam("outputs/angular_velocity", (bool&)_capabilities_.produces_angular_velocity);
  param_loader.loadParam("outputs/odometry", (bool&)_capabilities_.produces_odometry);

  param_loader.loadParam("topics/prefix", _simulator_prefix_);
  param_loader.loadParam("topics/simulator/odom", _topic_simulator_odom_);
  param_loader.loadParam("topics/simulator/imu", _topic_simulator_imu_);
  param_loader.loadParam("topics/simulator/rangefinder", _topic_simulator_rangefinder_);
  param_loader.loadParam("topics/simulator/actuators_cmd", _topic_simulator_actuators_cmd_);
  param_loader.loadParam("topics/simulator/control_group_cmd", _topic_simulator_control_group_cmd_);
  param_loader.loadParam("topics/simulator/attitude_rate_cmd", _topic_simulator_attitude_rate_cmd_);
  param_loader.loadParam("topics/simulator/attitude_cmd", _topic_simulator_attitude_cmd_);
  param_loader.loadParam("topics/simulator/acceleration_hdg_rate_cmd", _topic_simulator_acceleration_hdg_rate_cmd_);
  param_loader.loadParam("topics/simulator/velocity_hdg_rate_cmd", _topic_simulator_velocity_hdg_rate_cmd_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = node_;
  shopts.node_name          = "MrsSimulatorHwApi";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  //shopts.queue_size         = 10;
  //shopts.transport_hints    = rclcpp::TransportHints().tcpNoDelay();

  sh_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_odom_,
                                                           &Api::callbackOdom, this);

  sh_imu_ =
      mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_imu_, &Api::callbackImu, this);

  // | ----------------------- publishers ----------------------- |

  if (_capabilities_.accepts_actuator_cmd) {
    ph_actuators_cmd_ =
        mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiActuatorCmd>(node_, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_actuators_cmd_);
  }

  if (_capabilities_.accepts_control_group_cmd) {
    ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiControlGroupCmd>(
        node_, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_control_group_cmd_);
  }

  if (_capabilities_.accepts_attitude_rate_cmd) {
    ph_attitude_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>(
        node_, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_attitude_rate_cmd_);
  }

  if (_capabilities_.accepts_attitude_cmd) {
    ph_attitude_cmd_ =
        mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAttitudeCmd>(node_, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_attitude_cmd_);
  }

  if (_capabilities_.accepts_acceleration_hdg_rate_cmd) {
    ph_acceleration_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd>(
        node_, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_acceleration_hdg_rate_cmd_);
  }

  if (_capabilities_.accepts_velocity_hdg_rate_cmd) {
    ph_velocity_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>(
        node_, "/" + _simulator_prefix_ + "/" + _uav_name_ + "/" + _topic_simulator_velocity_hdg_rate_cmd_);
  }

  // | ------------------------- timers ------------------------- |

  {
    mrs_lib::TimerHandlerOptions timer_opts_start;

    timer_opts_start.node      = node_;
    timer_opts_start.autostart = true;

    std::function<void()> callback_fcn = std::bind(&Api::timerMain, this);
    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(10, clock_), callback_fcn);
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

mrs_msgs::msg::HwApiStatus Api::getStatus() {

  mrs_msgs::msg::HwApiStatus status;

  status.stamp = clock_->now();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

mrs_msgs::msg::HwApiCapabilities Api::getCapabilities() {

  _capabilities_.stamp = clock_->now();

  return _capabilities_;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> Api::callbackArming([[maybe_unused]] const bool& request) {

  std::stringstream ss;

  if (!request && !offboard_) {
    ss << "can not disarm, not in OFFBOARD mode";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
    return std::tuple(false, ss.str());
  }

  if (request) {

    armed_ = true;

    ss << "armed";
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
    return std::tuple(true, ss.str());

  } else {

    armed_ = false;

    ss << "disarmed";
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
    return std::tuple(true, ss.str());
  }
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> Api::callbackOffboard(void) {

  std::stringstream ss;

  if (!armed_) {
    ss << "Cannot switch to offboard, not armed.";
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str().c_str());
    return {false, ss.str()};
  }

  offboard_ = true;

  ss << "Offboard set";
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str().c_str());
  return {true, ss.str()};
}

/* destroy() //{ */

void Api::destroy() {
}

//}

// | --------------------- input callbacks -------------------- |

/* callbackActuatorCmd() //{ */

bool Api::callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_actuator_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting actuators cmd");

  ph_actuators_cmd_.publish(*msg);

  return true;
}

//}

/* callbackControlGroupCmd() //{ */

bool Api::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_control_group_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group cmd");

  ph_control_group_cmd_.publish(*msg);

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool Api::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate cmd");

  ph_attitude_rate_cmd_.publish(*msg);

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool Api::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_attitude_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude cmd");

  ph_attitude_cmd_.publish(*msg);

  return true;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool Api::callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_acceleration_hdg_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg rate cmd");

  ph_acceleration_hdg_rate_cmd_.publish(*msg);

  return true;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool Api::callbackAccelerationHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool Api::callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_velocity_hdg_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg rate cmd");

  ph_velocity_hdg_rate_cmd_.publish(*msg);

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool Api::callbackVelocityHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool Api::callbackPositionCmd([[maybe_unused]] const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void Api::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackOdom() */

void Api::callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting simulator odometry");

  auto odom = msg;

  {
    std::scoped_lock lock(mutex_status_);

    connected_ = true;
  }

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::msg::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = clock_->now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);

  // | -------------------- publish position -------------------- |

  if (_capabilities_.produces_position) {

    geometry_msgs::msg::PointStamped position;

    position.header.stamp    = odom->header.stamp;
    position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    position.point           = odom->pose.pose.position;

    common_handlers_->publishers.publishPosition(position);
  }

  // | ------------------- publish orientation ------------------ |

  if (_capabilities_.produces_orientation) {

    geometry_msgs::msg::QuaternionStamped orientation;

    orientation.header.stamp    = odom->header.stamp;
    orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    orientation.quaternion      = odom->pose.pose.orientation;

    common_handlers_->publishers.publishOrientation(orientation);
  }

  // | -------------------- publish velocity -------------------- |

  if (_capabilities_.produces_velocity) {

    geometry_msgs::msg::Vector3Stamped velocity;

    velocity.header.stamp    = odom->header.stamp;
    velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    velocity.vector          = odom->twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  // | ---------------- publish angular velocity ---------------- |

  if (_capabilities_.produces_angular_velocity) {

    geometry_msgs::msg::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp    = odom->header.stamp;
    angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    angular_velocity.vector          = odom->twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }

  // | -------------------- publish odometry -------------------- |

  if (_capabilities_.produces_odometry) {
    common_handlers_->publishers.publishOdometry(*odom);
  }

  // | ------------------ publish ground truth ------------------ |

  if (_capabilities_.produces_ground_truth) {
    common_handlers_->publishers.publishGroundTruth(*odom);
  }

  // | ---------------------- publish gnss ---------------------- |

  if (_capabilities_.produces_gnss) {

    double lat;
    double lon;

    mrs_lib::UTMtoLL(odom->pose.pose.position.y + _utm_y_, odom->pose.pose.position.x + _utm_x_, _utm_zone_, lat, lon);

    sensor_msgs::msg::NavSatFix gnss;

    gnss.header.stamp = odom->header.stamp;

    gnss.latitude  = lat;
    gnss.longitude = lon;
    gnss.altitude  = odom->pose.pose.position.z + _amsl_;

    common_handlers_->publishers.publishGNSS(gnss);
  }

  // | ------------------ publish amsl altitude ----------------- |

  if (_capabilities_.produces_altitude) {

    mrs_msgs::msg::HwApiAltitude altitude;

    altitude.stamp = odom->header.stamp;

    altitude.amsl = odom->pose.pose.position.z + _amsl_;

    common_handlers_->publishers.publishAltitude(altitude);
  }

  // | --------------------- publish heading -------------------- |

  if (_capabilities_.produces_magnetometer_heading) {

    double heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();

    mrs_msgs::msg::Float64Stamped hdg;

    hdg.header.stamp    = clock_->now();
    hdg.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    hdg.value           = heading;

    common_handlers_->publishers.publishMagnetometerHeading(hdg);
  }
}

//}

/* callbackImu() //{ */

void Api::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting IMU");

  if (_capabilities_.produces_imu) {

    common_handlers_->publishers.publishIMU(*msg);
  }
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void Api::timerMain() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "main timer spinning");

  publishBatteryState();

  publishRC();

  publishDistanceSensor();
}

//}

// | ------------------------- methods ------------------------ |

/* publishBatteryState() //{ */

void Api::publishBatteryState(void) {

  if (_capabilities_.produces_battery_state) {

    sensor_msgs::msg::BatteryState msg;

    msg.capacity = 100;
    msg.current  = 10.0;
    msg.voltage  = 15.8;
    msg.charge   = 0.8;

    common_handlers_->publishers.publishBatteryState(msg);
  }
}

//}

/* publishRC() //{ */

void Api::publishRC(void) {

  if (_capabilities_.produces_rc_channels) {

    mrs_msgs::msg::HwApiRcChannels rc;

    rc.stamp = clock_->now();

    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);

    common_handlers_->publishers.publishRcChannels(rc);
  }
}

//}

/* publishDistanceSensor() //{ */

void Api::publishDistanceSensor(void) {

  if (_capabilities_.produces_distance_sensor) {

    sensor_msgs::msg::Range msg;

    msg.header.stamp = clock_->now();

    msg.range     = -1;
    msg.min_range = 0.1;
    msg.max_range = 30;

    common_handlers_->publishers.publishDistanceSensor(msg);
  }
}

//}

/* MrsUavHwApi() //{ */

void Api::timeoutInputs(void) {
}

//}

}  // namespace mrs_uav_coppelia_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_coppelia_api::Api, mrs_uav_hw_api::MrsUavHwApi)
