#pragma once
#include <types/StateTriplet.h>
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
    TYPE_ACC
  } Type;

  StateTriplet initialState;
  StateTriplet targetState;
  double estimated_time_s;
  double max_velocity;
  double costs = 0;
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
           PlanItem::Type type = PlanItem::TYPE_POS_VEL_ACC)
  {
    this->initialState.set(initialState);
    this->targetState = targetState;
    this->estimated_time_s = estimated_time_s;
    this->planning_type = type;
  };

  inline void setInitialState(StateTriplet t)
  {
    initialState.set(t);
  };
  inline void setTargetState(StateTriplet t, PlanItem::Type type = PlanItem::TYPE_POS_VEL_ACC,
                             double estimated_time_s = 0.0, float max_vel = 1.0f)
  {
    targetState.set(t);
    planning_type = type;
    max_velocity = max_vel;
    this->estimated_time_s = estimated_time_s;
  };

  inline void setInitialFromTarget(PlanItem item)
  {
    setInitialState(item.targetState);
  }

  inline double getDistance()
  {
    return (targetState.pos - initialState.pos).GetNorm2();
  };

};

inline std::ostream& operator<<(std::ostream& os, const PlanItem& i) {
    os << "InitialState: " << i.initialState << std::endl;
    os << "TargetState : " << i.targetState << std::endl;
    os << "Time        : " << i.estimated_time_s <<"s" << std::endl;
    os << "Max.Velocity: " << i.max_velocity <<"m/s" << std::endl;
    os << "Type        : " << i.planning_type << std::endl;
    return os;
};

}