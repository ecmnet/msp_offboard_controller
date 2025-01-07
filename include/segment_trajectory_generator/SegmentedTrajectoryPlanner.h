#pragma once
#include <segment_trajectory_generator/MSPRapidTrajectoryGenerator.h>
#include <types/PlanItem.h>
#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <boost/math/tools/minima.hpp>

#define ACCELERATION_PHASE_SECS 2.5f
#define DECELERATION_PHASE_SECS 2.0f
#define MAX_VELOCITY 1.0f

namespace msp
{


class SegmentedTrajectoryPlanner
{
public:

  /*! Generated an optimized execution plan for a direct path 
   *
   * @param initialState: The initial state of the vehicle
   * @param target_pos  : The target position of the vehicle
   * @param max_velocity: The cruising velocity after acceleration
   */
   std::queue<PlanItem> createOptimizedDirectPathPlan(const StateTriplet initialState, Vec3 target_pos, float max_velocity);

  /*! Returns a created execution plan 
   *
   * @return A Vector containing all segments (PlanItems) representing the plan
   */
  std::queue<PlanItem> getPlan(void);

private:

  /*! Creates a trajectory with 3 phases 
   *
   * @param initialState   : The initial state of the vehicle
   * @param target_pos     : The target position of the vehicle
   * @param max_velocity   : The cruising velocity after acceleration
   * @param angle_variation: Variing the target angle of the aceleration phase (for optimizer)
   * @return Total costs of the trajectory
   */
  double createDirectPathPlan(const StateTriplet initialState, const Vec3 target_pos, const float max_velocity,
                             double angle_variation = 0);

  StateTriplet initialState;
  std::queue<PlanItem> plan;

  float max_velocity;

  msp::MSPRapidTrajectoryGenerator segmentPlanner = msp::MSPRapidTrajectoryGenerator();
};

}  // namespace msp
