
#include <msp_offboard_controller/msp_offboard_controller.hpp>
#include <segment_trajectory_generator/SegmentedTrajectoryPlanner.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/log_message.hpp>

/**
 * @brief MSP offboard controller
 */

class MSPOffboardControllerNode : public rclcpp::Node
{
public:
  explicit MSPOffboardControllerNode() : Node("MSPOffboardController")
  {
	RCLCPP_INFO(this->get_logger(), "Offboard controller started");
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	msp_cmd_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(MSP_CMD_PUB, qos);
	setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(MSP_TRJ_PUB, qos);
	message_publisher_ = this->create_publisher<px4_msgs::msg::LogMessage>(MSP_LOG_PUB, qos);

	status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		MSP_STATUS_SUB, qos, [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
		  // if offboard left externally, switch state to idle
		  if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
			  msg->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
			  state_ == State::execute_segment)
		  {
			log_message("[msp] Offboard stopped externally", MAV_SEVERITY_NOTICE);
			state_ = State::idle;
		  }
		  nav_state = msg->nav_state;
		});

	local_pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		MSP_POS_SUB, qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
		  current_state.set(msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->ax, msg->ay, msg->az);
		  current_yaw = msg->heading;
		  if (!initialized)
		  {
			initialized = true;
			log_message("[msp] MSPOffboardController ready", MAV_SEVERITY_INFO);
		  }
		});

	msp_cmd_subscription_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
		MSP_CMD_SUB, qos, [this](const px4_msgs::msg::VehicleCommand::UniquePtr msg) {
		  switch (msg->command)
		  {
			case MSP_CMD_OFFBOARD_SETLOCALPOS:

			  target_pos.x = msg->param1;
			  target_pos.y = msg->param2;
			  if (std::isfinite(msg->param3))
				target_pos.z = msg->param3;
			  else
				target_pos.z = current_state.pos.z;

			  //current_plan = planner_.createCirclePathPlan(current_state, target_pos, 1.0f, 2.5f, 3);
			  current_plan = planner_.createOptimizedDirectPathPlan(current_state, msp::StateTriplet(target_pos), MAX_VELOCITY);
			  std::cout << current_plan << std::endl;
			  state_ = State::offboard_requested;
			  break;

			default:
			  log_message("[msp] Unknown MSP command", MAV_SEVERITY_ERROR);
		  }
		});

	timer_ = this->create_wall_timer(std::chrono::milliseconds(OFFBOARD_RATE),
									 std::bind(&MSPOffboardControllerNode::offboard_worker, this));
  }

private:
  void offboard_worker()
  {
	switch (state_)
	{
	  case State::idle:
		this->counter = 0;
		break;

	  case State::offboard_requested:

		this->elapsed_s = 0;
		this->offset_s = 0;
		this->start_us = 0;

		if (current_plan.empty())
		{
		  RCLCPP_INFO(this->get_logger(), "No execution plan available. Request ignored.");
		  state_ = State::idle;
		  return;
		}

		RCLCPP_INFO(this->get_logger(), "Offboard control requested");

		// Initial checks
		if (!initialized || (nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER &&
							 nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD))
		{
		  this->log_message("[msp] Offboard request rejected.", MAV_SEVERITY_NOTICE);
		  state_ = State::idle;
		  return;
		}
		state_ = State::wait_for_stable_offboard_mode;
		break;

	  case State::wait_for_stable_offboard_mode:

		// Already in offboard mode?
		if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
		{
		  state_ = State::plan_next_segment;
		  return;
		}

		// Send some setpoints before switching to offboard
		if (++counter == 5)
		{
		  this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		  RCLCPP_INFO(this->get_logger(), "Offboard execution started.");
		  state_ = State::plan_next_segment;
		}
		this->sendSetpoint(current_state);
		break;

	  case State::plan_next_segment:

		if (current_plan.empty())
		{
		  state_ = State::target_reached;
		  return;
		}
		current_segment = current_plan.next();
		executor_.generate(current_segment);
		start_us = this->get_clock()->now().nanoseconds() / 1000L;
		state_ = State::execute_segment;
		// RCLCPP_INFO(this->get_logger(), "Next segment execution started.");
		break;

	  case State::execute_segment:

		elapsed_s = (this->get_clock()->now().nanoseconds() / 1000L - start_us) / 1e6;

		if (elapsed_s > current_segment.estimated_time_s)
		{
		  state_ = State::plan_next_segment;
		  return;
		}

		executor_.getSetpointAt(elapsed_s, current_setpoint);
		sendSetpoint(current_setpoint);
		break;

	  case State::target_reached:

		this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
		// TODO: Run callback
		this->log_message("[msp] Target reached", MAV_SEVERITY_INFO);
		state_ = State::idle;
		break;
	}
	counter++;
  }

  void sendSetpoint(msp::StateTriplet setpoint)
  {
	auto message = px4_msgs::msg::TrajectorySetpoint();

	message.position[0] = setpoint.pos.x;
	message.position[1] = setpoint.pos.y;
	message.position[2] = setpoint.pos.z;

	message.velocity[0] = setpoint.vel.x;
	message.velocity[1] = setpoint.vel.y;
	message.velocity[2] = setpoint.vel.z;

	message.acceleration[0] = setpoint.acc.x;
	message.acceleration[1] = setpoint.acc.y;
	message.acceleration[2] = setpoint.acc.z;

	if (setpoint.vel.GetNorm2() > MIN_YAW_FOLLOW_VELOCITY)
	  message.yaw = setpoint.vel.getXYAngle();
	else
	  message.yaw = std::numeric_limits<float>::quiet_NaN();

	message.timestamp = this->get_clock()->now().nanoseconds() / 1000L;
	setpoint_publisher_->publish(message);
  }

  void publish_vehicle_command(const uint16_t command, const float param1 = 0, const float param2 = 0,
							   const float param3 = 0)
  {
	auto msg = px4_msgs::msg::VehicleCommand();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msp_cmd_publisher_->publish(msg);
  }

  void log_message(const std::string msg, const uint8_t severity)
  {
	auto message = px4_msgs::msg::LogMessage();
	std::copy_n(msg.begin(), std::min(msg.size(), message.text.size()), message.text.begin());
	message.set__severity(severity);
	message_publisher_->publish(message);
	RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr msp_cmd_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr msp_cmd_publisher_;
  rclcpp::Publisher<px4_msgs::msg::LogMessage>::SharedPtr message_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  msp::SegmentedTrajectoryPlanner planner_ = msp::SegmentedTrajectoryPlanner();
  msp::MSPRapidTrajectoryGenerator executor_ = msp::MSPRapidTrajectoryGenerator();

  long start_us = 0;
  double elapsed_s = 0;
  double offset_s = 0;
  bool initialized = false;
  uint8_t nav_state;

  // Target position
  Vec3 target_pos = Vec3(0, 0, 0);

  // Current state
  msp::StateTriplet current_state = msp::StateTriplet();
  float current_yaw = 0;

  // Current setpoint
  msp::StateTriplet current_setpoint = msp::StateTriplet();

  // Current plan: A vector of PlanItems
  msp::MSPTrajectory current_plan;

  // Current segment
  msp::PlanItem current_segment;

  // Offboard states
  enum class State
  {
	idle,
	offboard_requested,
	wait_for_stable_offboard_mode,
	plan_next_segment,
	execute_segment,
	target_reached,
  } state_ = State::idle;

  long counter;
};

int main(int argc, char* argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MSPOffboardControllerNode>());
  rclcpp::shutdown();
  return 0;
}
