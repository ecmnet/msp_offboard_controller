
#include <msp_offboard_controller/msp_offboard_controller.hpp>
#include <msp_controller/msp_node_base.hpp>
#include <segment_trajectory_generator/SegmentedTrajectoryPlanner.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <msp_msgs/msg/trajectory.hpp>
#include <msp_msgs/srv/trajectory_check.hpp>
// #include <msp_msgs/msg/heartbeat.hpp>

/**
 * @brief MSP offboard controller
 */

class MSPOffboardControllerNode : public msp::MSPNodeBase // public rclcpp::Node
{
public:
	explicit MSPOffboardControllerNode() : msp::MSPNodeBase("MSPOffboardController", MSP_COMP_OFFBOARD)
	{
		RCLCPP_INFO(this->get_logger(), "Offboard controller started");
		auto qos = this->getQos();

		setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(MSP_TRJ_PUB, qos);
		trajectory_publisher_ = this->create_publisher<msp_msgs::msg::Trajectory>("msp/in/trajectory", qos);
		// heartbeat_publisher_ = this->create_publisher<msp_msgs::msg::Heartbeat>("/msp/in/heartbeat", qos);

		msp_trajectory_check_client = this->create_client<msp_msgs::srv::TrajectoryCheck>("/msp/in/trajectory_check");

		status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			MSP_STATUS_SUB, qos, [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg)
			{
		  // if offboard left externally, switch state to idle
		  if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
			  msg->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
			  state_ == State::execute_segment) 
		  {
			sendTrajectory(current_segment);
			log_message("[msp] Offboard stopped externally", MAV_SEVERITY_NOTICE);
			state_ = State::idle;
		  }
		  nav_state = msg->nav_state; });

		local_pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			MSP_POS_SUB, qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
			{
		  current_state.set(msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->ax, msg->ay,msg->az);
		                 //   msg->acceleration[0], msg->acceleration[1], msg->acceleration[2]);

		  current_yaw = msg->heading;

		  if (!initialized)
		  {
			initialized = true;
			log_message("[msp] MSPOffboardController ready", MAV_SEVERITY_INFO);
		  } });

		timer_ = this->create_wall_timer(std::chrono::milliseconds(OFFBOARD_RATE),
										 std::bind(&MSPOffboardControllerNode::offboard_worker, this));

		// hb_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000),
		// 									std::bind(&MSPOffboardControllerNode::send_heartbeat, this));
	}

	void receive_msp_command(const std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request,
							 std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> response) override
	{
		switch (request->request.command)
		{
		case MSP_CMD::MSP_CMD_OFFBOARD_SETLOCALPOS:

			target_pos.x = request->request.param1;
			target_pos.y = request->request.param2;
			if (std::isfinite(request->request.param3))
				target_pos.z = request->request.param3;
			else
				target_pos.z = current_state.pos.z;
			current_plan = msp::MSPTrajectory();
			current_plan.addAll(planner_.createOptimizedDirectPathPlan(current_state, target_pos, MAX_VELOCITY, 2.5f));
			// current_plan.addAll(planner_.createCirclePathPlan(current_plan.getLastState(), 1.0f, 10.0f, 1));
			// std::cout << current_plan << std::endl;
			state_ = State::offboard_requested;
			break;
		}
	}

private:
	void offboard_worker()
	{
		switch (state_)
		{
		case State::idle:
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
			this->counter = 0;
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
			if (++this->counter == 10)
			{
				this->send_px4_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
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

			// // If the item is a new path, update initial state
			// if (current_segment.isFirst())
			current_segment.setInitialState(current_state);

			executor_.generate(&current_segment);
            checkTrajectory(current_segment);

			start_us = this->get_clock()->now().nanoseconds() / 1000L;
			state_ = State::execute_segment;
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
			sendTrajectory(current_segment, elapsed_s);
			break;

		case State::target_reached:

			sendTrajectory(current_segment);
			this->send_px4_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
			// TODO: Run callback
			this->log_message("[msp] Target reached", MAV_SEVERITY_INFO);
			state_ = State::idle;
			break;
		}
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

	void sendTrajectory(msp::PlanItem item, double elapsed_s = -1.0)
	{

		auto message = msp_msgs::msg::Trajectory();

		message.id = 1;
		message.done_secs = elapsed_s;
		message.total_secs = item.estimated_time_s;

		for (int i = 0; i < 3; i++)
		{
			message.alpha[i] = float(item.alpha[i]);
			message.beta[i] = float(item.beta[i]);
			message.gamma[i] = float(item.gamma[i]);

			message.pos0[i] = item.initialState.pos[i];
			message.vel0[i] = item.initialState.vel[i];
			message.acc0[i] = item.initialState.acc[i];
		}

		message.timestamp = this->get_clock()->now().nanoseconds() / 1000L;
		trajectory_publisher_->publish(message);
	}

	void checkTrajectory(msp::PlanItem item)
	{
		auto request = std::make_shared<msp_msgs::srv::TrajectoryCheck::Request>();
		auto message = msp_msgs::msg::Trajectory();

		//TODO: Rotate params to BodyFrame

		message.id = 1;
		message.done_secs = elapsed_s;
		message.total_secs = item.estimated_time_s;

		for (int i = 0; i < 3; i++)
		{
			message.alpha[i] = float(item.alpha[i]);
			message.beta[i] =  float(item.beta[i]);
			message.gamma[i] = float(item.gamma[i]);

			message.pos0[i] = item.initialState.pos[i];
			message.vel0[i] = item.initialState.vel[i];
			message.acc0[i] = item.initialState.acc[i];
		}

		message.timestamp = this->get_clock()->now().nanoseconds() / 1000L;

		request->request = message;
		msp_trajectory_check_client->async_send_request(request,std::bind(&MSPOffboardControllerNode::handleCollisionCheckResult, this,std::placeholders::_1));
	}

	void handleCollisionCheckResult(rclcpp::Client<msp_msgs::srv::TrajectoryCheck>::SharedFuture future) {
		auto response = future.get();
		if(response->reply.status == 0) {
		   state_ = State::idle;
		   this->send_px4_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
           this->log_message("[msp] Emergency stop. Collision expected.", MAV_SEVERITY_ALERT);
		}
	}

	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;

	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
	rclcpp::Publisher<msp_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

	rclcpp::Client<msp_msgs::srv::TrajectoryCheck>::SharedPtr msp_trajectory_check_client;

	//   rclcpp::Publisher<msp_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;

	rclcpp::TimerBase::SharedPtr timer_;
	//   rclcpp::TimerBase::SharedPtr hb_timer_;

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

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MSPOffboardControllerNode>());
	rclcpp::shutdown();
	return 0;
}
