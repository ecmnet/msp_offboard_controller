#pragma once
#include <msp_offboard_controller/types/PlanItem.h>
#include <msp_msgs/msg/trajectory_plan_item.hpp>

namespace msp::ros2::convert {

 msp_msgs::msg::TrajectoryPlanItem toTrajectoryPlanItemMessage(msp::PlanItem p)
  {
    msp_msgs::msg::TrajectoryPlanItem item;
    item.estimated_time_s = p.estimated_time_s;
    item.planning_type = p.planning_type;
    item.max_velocity = p.max_velocity;
    item.costs = p.costs;
    item.is_first = p.is_first;
    item.initial_pos = {p.initialState.pos.x, p.initialState.pos.y, p.initialState.pos.z};
    item.initial_vel = {p.initialState.vel.x, p.initialState.vel.y, p.initialState.vel.z};
    item.initial_acc = {p.initialState.acc.x, p.initialState.acc.y, p.initialState.acc.z};
    item.target_pos  = {p.targetState.pos.x, p.targetState.pos.y, p.targetState.pos.z};
    item.target_vel  = {p.targetState.vel.x, p.targetState.vel.y, p.targetState.vel.z};
    item.target_acc  = {p.targetState.acc.x, p.targetState.acc.y, p.targetState.acc.z};
    item.alpha = {p.alpha.x, p.alpha.y, p.alpha.z};
    item.beta  = {p.beta.x, p.beta.y, p.beta.z};
    item.gamma = {p.gamma.x, p.gamma.y, p.gamma.z};
    return item;
  }

   msp::PlanItem fromTrajectoryPlanItemMessage(msp_msgs::msg::TrajectoryPlanItem item)
  {
    msp::PlanItem p;
    p.estimated_time_s = item.estimated_time_s;
    p.planning_type = static_cast<msp::PlanItem::Type>(item.planning_type);
    p.max_velocity = item.max_velocity;
    p.costs = item.costs;
    p.is_first = item.is_first;
    p.initialState.pos = {item.initial_pos[0], item.initial_pos[1], item.initial_pos[2]};
    p.initialState.vel = {item.initial_vel[0], item.initial_vel[1], item.initial_vel[2]};
    p.initialState.acc = {item.initial_acc[0], item.initial_acc[1], item.initial_acc[2]};
    p.targetState.pos = {item.target_pos[0], item.target_pos[1], item.target_pos[2]};
    p.targetState.vel = {item.target_vel[0], item.target_vel[1], item.target_vel[2]};
    p.targetState.acc = {item.target_acc[0], item.target_acc[1], item.target_acc[2]};
    p.alpha = {item.alpha[0], item.alpha[1], item.alpha[2]};
    p.beta  = {item.beta[0], item.beta[1], item.beta[2]};
    p.gamma = {item.gamma[0], item.gamma[1], item.gamma[2]};
    return p;
  }

}