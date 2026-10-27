#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

class MissionManager
{
public:
  MissionManager()
    : private_nh_("~"), state_(State::READY), have_state_(false), have_pose_(false)
  {
    private_nh_.param("setpoint_topic", setpoint_topic_, std::string("/mavros/setpoint_position/local"));
    private_nh_.param("state_topic", state_topic_, std::string("/mission/state"));
    private_nh_.param("offboard_mode", offboard_mode_, std::string("OFFBOARD"));
    private_nh_.param("fallback_mode", fallback_mode_, std::string("POSCTL"));
    private_nh_.param("takeoff_altitude", takeoff_altitude_, 1.5);
    private_nh_.param("search_distance", search_distance_, 11.0);
    private_nh_.param("arrival_tolerance", arrival_tolerance_, 0.35);
    private_nh_.param("pose_timeout", pose_timeout_, 0.5);
    private_nh_.param("prestream_duration", prestream_duration_, 1.5);
    private_nh_.param("setpoint_rate", setpoint_rate_, 20.0);
    private_nh_.param("mode_retry_period", mode_retry_period_, 1.0);
    private_nh_.param("arming_retry_period", arming_retry_period_, 1.0);

    state_sub_ = nh_.subscribe("/mavros/state", 10, &MissionManager::stateCallback, this);
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &MissionManager::poseCallback, this);
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(setpoint_topic_, 10);
    mission_state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 10, true);
    start_service_ = nh_.advertiseService("/mission/start", &MissionManager::startCallback, this);
    return_service_ = nh_.advertiseService("/mission/return", &MissionManager::returnCallback, this);
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    timer_ = nh_.createTimer(ros::Duration(1.0 / setpoint_rate_), &MissionManager::timerCallback, this);
    publishState();
    ROS_WARN("Mission manager is idle. Start explicitly with: rosservice call /mission/start");
  }

private:
  enum class State
  {
    READY,
    PRESTREAM,
    REQUEST_OFFBOARD,
    ARMING,
    TAKEOFF,
    SEARCH,
    SEARCH_COMPLETE,
    RETURN,
    RETURN_COMPLETE,
    ABORTED
  };

  static const char* stateName(State state)
  {
    switch (state) {
      case State::READY: return "READY";
      case State::PRESTREAM: return "PRESTREAM";
      case State::REQUEST_OFFBOARD: return "REQUEST_OFFBOARD";
      case State::ARMING: return "ARMING";
      case State::TAKEOFF: return "TAKEOFF";
      case State::SEARCH: return "SEARCH";
      case State::SEARCH_COMPLETE: return "SEARCH_COMPLETE";
      case State::RETURN: return "RETURN";
      case State::RETURN_COMPLETE: return "RETURN_COMPLETE";
      case State::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message)
  {
    current_state_ = *message;
    have_state_ = true;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    current_pose_ = *message;
    last_pose_receipt_ = ros::Time::now();
    have_pose_ = true;
  }

  bool poseIsFresh() const
  {
    return have_pose_ && !last_pose_receipt_.isZero() &&
           (ros::Time::now() - last_pose_receipt_).toSec() <= pose_timeout_;
  }

  bool startCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (state_ != State::READY) {
      response.success = false;
      response.message = "Mission can only start from READY.";
      return true;
    }
    if (!have_state_ || !current_state_.connected || !poseIsFresh()) {
      response.success = false;
      response.message = "FCU must be connected and local pose must be fresh.";
      return true;
    }

    origin_ = current_pose_.pose.position;
    target_pose_ = current_pose_;
    target_pose_.pose.position.z = origin_.z + takeoff_altitude_;
    transition(State::PRESTREAM);
    response.success = true;
    response.message = "Mission accepted; streaming local position setpoints before OFFBOARD.";
    return true;
  }

  bool returnCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (state_ != State::SEARCH_COMPLETE) {
      response.success = false;
      response.message = "Return is available only after the search waypoint is reached.";
      return true;
    }

    target_pose_.pose.position.x = origin_.x;
    target_pose_.pose.position.y = origin_.y;
    transition(State::RETURN);
    response.success = true;
    response.message = "Returning to the local takeoff position; landing is not commanded.";
    return true;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    const bool mission_active = state_ != State::READY;
    const bool expects_offboard = state_ == State::ARMING || state_ == State::TAKEOFF ||
                                  state_ == State::SEARCH || state_ == State::SEARCH_COMPLETE ||
                                  state_ == State::RETURN;
    if (mission_active && state_ != State::ABORTED &&
        (!have_state_ || !current_state_.connected || !poseIsFresh() ||
         (expects_offboard && current_state_.mode != offboard_mode_))) {
      transition(State::ABORTED);
      ROS_ERROR("Mission safety condition failed: FCU disconnected, local pose stale, or OFFBOARD exited.");
    }

    if (state_ == State::READY) {
      return;
    }

    publishSetpoint();

    if (state_ == State::PRESTREAM) {
      if ((ros::Time::now() - state_started_).toSec() >= prestream_duration_) {
        transition(State::REQUEST_OFFBOARD);
      }
      return;
    }

    if (state_ == State::REQUEST_OFFBOARD) {
      if (have_state_ && current_state_.mode == offboard_mode_) {
        transition(State::ARMING);
      } else if (retryDue(last_mode_request_, mode_retry_period_)) {
        mavros_msgs::SetMode request;
        request.request.custom_mode = offboard_mode_;
        if (mode_client_.call(request) && request.response.mode_sent) {
          ROS_INFO("Requested flight mode %s", offboard_mode_.c_str());
        } else {
          ROS_WARN_THROTTLE(2.0, "PX4 did not accept mode request %s", offboard_mode_.c_str());
        }
        last_mode_request_ = ros::Time::now();
      }
      return;
    }

    if (state_ == State::ARMING) {
      if (have_state_ && current_state_.armed) {
        transition(State::TAKEOFF);
      } else if (retryDue(last_arming_request_, arming_retry_period_)) {
        mavros_msgs::CommandBool request;
        request.request.value = true;
        if (!arming_client_.call(request) || !request.response.success) {
          ROS_WARN_THROTTLE(2.0, "PX4 did not accept arming request");
        }
        last_arming_request_ = ros::Time::now();
      }
      return;
    }

    if (state_ == State::TAKEOFF && reachedTarget3d()) {
      target_pose_.pose.position.x = origin_.x + search_distance_;
      target_pose_.pose.position.y = origin_.y;
      transition(State::SEARCH);
    } else if (state_ == State::SEARCH && reachedSearchDistance()) {
      transition(State::SEARCH_COMPLETE);
      ROS_INFO("Search waypoint reached: local horizontal displacement %.2f m", horizontalDisplacement());
    } else if (state_ == State::RETURN && reachedTarget3d()) {
      transition(State::RETURN_COMPLETE);
    } else if (state_ == State::ABORTED) {
      requestFallbackMode();
    }
  }

  bool retryDue(const ros::Time& last_request, double retry_period) const
  {
    return last_request.isZero() || (ros::Time::now() - last_request).toSec() >= retry_period;
  }

  bool reachedTarget3d() const
  {
    const auto& position = current_pose_.pose.position;
    const double dx = position.x - target_pose_.pose.position.x;
    const double dy = position.y - target_pose_.pose.position.y;
    const double dz = position.z - target_pose_.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) <= arrival_tolerance_;
  }

  double horizontalDisplacement() const
  {
    const double dx = current_pose_.pose.position.x - origin_.x;
    const double dy = current_pose_.pose.position.y - origin_.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  bool reachedSearchDistance() const
  {
    return horizontalDisplacement() >= search_distance_ - arrival_tolerance_ && reachedTarget3d();
  }

  void requestFallbackMode()
  {
    if (!have_state_ || !current_state_.connected || fallback_mode_.empty()) {
      return;
    }
    if (current_state_.mode == fallback_mode_) {
      return;
    }
    if (!retryDue(last_mode_request_, mode_retry_period_)) {
      return;
    }

    mavros_msgs::SetMode request;
    request.request.custom_mode = fallback_mode_;
    if (mode_client_.call(request) && request.response.mode_sent) {
      ROS_WARN("Requested fallback flight mode %s", fallback_mode_.c_str());
    } else {
      ROS_ERROR_THROTTLE(2.0, "Fallback mode request %s failed; holding last setpoint", fallback_mode_.c_str());
    }
    last_mode_request_ = ros::Time::now();
  }

  void publishSetpoint()
  {
    target_pose_.header.stamp = ros::Time::now();
    if (target_pose_.header.frame_id.empty()) {
      target_pose_.header.frame_id = current_pose_.header.frame_id;
    }
    setpoint_pub_.publish(target_pose_);
  }

  void transition(State next)
  {
    if (state_ == next) {
      return;
    }
    state_ = next;
    state_started_ = ros::Time::now();
    std_msgs::String message;
    message.data = stateName(state_);
    mission_state_pub_.publish(message);
    ROS_INFO("Mission state: %s", message.data.c_str());
  }

  void publishState()
  {
    std_msgs::String message;
    message.data = stateName(state_);
    mission_state_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher mission_state_pub_;
  ros::ServiceServer start_service_;
  ros::ServiceServer return_service_;
  ros::ServiceClient mode_client_;
  ros::ServiceClient arming_client_;
  ros::Timer timer_;

  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped target_pose_;
  geometry_msgs::Point origin_;
  State state_;
  ros::Time state_started_;
  ros::Time last_pose_receipt_;
  ros::Time last_mode_request_;
  ros::Time last_arming_request_;
  bool have_state_;
  bool have_pose_;
  std::string setpoint_topic_;
  std::string state_topic_;
  std::string offboard_mode_;
  std::string fallback_mode_;
  double takeoff_altitude_;
  double search_distance_;
  double arrival_tolerance_;
  double pose_timeout_;
  double prestream_duration_;
  double setpoint_rate_;
  double mode_retry_period_;
  double arming_retry_period_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_manager");
  MissionManager mission_manager;
  ros::spin();
  return 0;
}