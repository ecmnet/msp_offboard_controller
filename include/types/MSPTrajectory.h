#pragma once
#include <rclcpp/rclcpp.hpp>
#include <types/PlanItem.h>
#include <queue>

namespace msp
{

class MSPTrajectory : public std::queue<PlanItem>
{
public:

  inline void addAll(std::queue<PlanItem> segments)
  {
    while (!segments.empty())
    {
      this->push(segments.front());
      segments.pop();
    }
  }

  inline StateTriplet getLastState() {
    PlanItem item = this->back();
    return item.targetState;
  }

  inline PlanItem next(StateTriplet replace_initial_state)
  {
    PlanItem i = this->front();
    i.initialState.set(replace_initial_state);
    this->pop();
    return i;
  }

  inline PlanItem next()
  {
    PlanItem i = this->front();
    this->pop();
    return i;
  }

  double total_costs = 0;
  
};

inline std::ostream& operator<<(std::ostream& os, const MSPTrajectory& i) {
    MSPTrajectory copy = i;
    int k = 0;
    os << "Planned trajectory: " << std::endl;
    os << "===========================================================" << std::endl;
    while (!copy.empty()) {
        std::cout << "Segment " << ++k << std::endl;
        std::cout << copy.front() << std::endl;
        copy.pop();
    }
    os << "===========================================================" << std::endl;
    return os;
};

}  // namespace msp