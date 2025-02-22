#pragma once
#include <msp_offboard_controller/types/StateTriplet.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

//! PlanItem class
/*!
 * Represents a planning item
 */

namespace msp {

class PlanItem
{
public:
  typedef enum
  {
    TYPE_POS,
    TYPE_POS_VEL,
    TYPE_POS_VEL_ACC,
    TYPE_VEL,
    TYPE_VEL_ACC,
    TYPE_ACC,
    TYPE_YAW_ONLY
  } Type;

  StateTriplet initialState; //! Initialstate (usually the current state)
  StateTriplet targetState;  //! Target state
  double estimated_time_s;   //! Estimated time of the segment in seconds
  double max_velocity;       //! Used maxim velocity
  double costs = 0;          //! Costs of the segment
  bool   is_first;           //! First item of a path (used to do a initial state update with current)
  Vec3   alpha;              //! Trajectory parameter alpha
  Vec3   beta;               //! Trajectory parameter beta
  Vec3   gamma;              //! Trajectory parameter gamma

  Type planning_type = PlanItem::TYPE_POS_VEL_ACC;

  PlanItem(void) : initialState(), targetState()
  {
    estimated_time_s = 0;
    planning_type = PlanItem::TYPE_POS_VEL_ACC;
    max_velocity = 1.0f;
  };

  PlanItem(StateTriplet initialState)
  {
    this->initialState.set(initialState);
  };

  PlanItem(StateTriplet initialState, StateTriplet targetState, double estimated_time_s,
           PlanItem::Type type = PlanItem::TYPE_POS_VEL_ACC, bool is_first = false )
  {
    this->initialState.set(initialState);
    this->targetState = targetState;
    this->estimated_time_s = estimated_time_s;
    this->planning_type = type;
    this->is_first = is_first;
  };

  inline void setInitialState(StateTriplet t)
  {
    initialState.set(t);
  };

  inline void setTargetState(StateTriplet t, PlanItem::Type type = PlanItem::TYPE_POS_VEL_ACC,
                             double estimated_time_s = 0.0, float max_vel = 1.0f, bool is_first = false )
  {
    targetState.set(t);
    planning_type = type;
    max_velocity = max_vel;
    this->estimated_time_s = estimated_time_s;
    this->is_first = is_first;
  };

  inline void setInitialFromTarget(PlanItem item)
  {
    setInitialState(item.targetState);
  }

  inline double getDistance()
  {
    return (targetState.pos - initialState.pos).GetNorm2();
  };

  inline bool isFirst() {
    return this->is_first;
  };


};



inline std::ostream& operator<<(std::ostream& os, const PlanItem& i) {
    os << "InitialState: " << i.initialState << std::endl;
    os << "TargetState : " << i.targetState << std::endl;
    os << "Time        : " << i.estimated_time_s <<"s" << std::endl;
    os << "Max.Velocity: " << i.max_velocity <<"m/s" << std::endl;
    os << "Type        : " << i.planning_type << std::endl;
    os << "Is first    : " << i.is_first << std::endl;
    return os;
};

}