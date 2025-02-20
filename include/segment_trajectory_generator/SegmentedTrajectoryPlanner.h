#pragma once
#include <segment_trajectory_generator/MSPRapidTrajectoryGenerator.h>
#include <types/PlanItem.h>
#include <types/MSPTrajectory.h>
#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <boost/math/tools/minima.hpp>

#define MIN_VELOCITY 0.1f
#define MAX_VELOCITY 1.0f
#define MAX_YAWSPEED 0.75f

#define ACCELERATION_PHASE_SECS 2.5f
#define DECELERATION_PHASE_SECS 2.5f

#define MIN_CRUISE_TIME 0.5f

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
     * @return Execution plan
     */
    MSPTrajectory createOptimizedDirectPathPlan(const StateTriplet initialState,
                                                const StateTriplet targetState,
                                                float max_velocity,
                                                float deceleration_phase_secs = DECELERATION_PHASE_SECS);

    /*! Generated an optimized execution plan for a direct path
     *
     * @param initialState: The initial state of the vehicle
     * @param max_velocity: The cruising velocity after acceleration
     * @param radius      : The radius of the circle
     * @param turns       : Turns to be executed
     * @return Execution plan
     */
    MSPTrajectory createCirclePathPlan(const StateTriplet initialState, float max_velocity, float radius,
                                       int turns = 1);

    /*! Generated an optimized execution plan for a direct path
     *
     * @param initialState: The initial state of the vehicle
     * @param targetState : The target state of the vehicle
     * @param max_velocity: The cruising velocity after acceleration
     * @return Execution plan containing one segment
     */
    MSPTrajectory createPlanSegment(const StateTriplet initialState, StateTriplet targetState,
                                    const PlanItem::Type type, const float max_velocity);

    /*! Returns a created execution plan
     *
     * @return A Vector containing all segments (PlanItems) representing the plan
     */
    MSPTrajectory getPlan(void);

  private:
    /*! Creates a trajectory with 3 phases
     *
     * @param initialState   : The initial state of the vehicle
     * @param targetState    : The target state of the vehicle
     * @param max_velocity   : The cruising velocity after acceleration
     * @param angle_variation: Variing the target angle of the aceleration phase (for optimizer)
     * @return Total costs of the trajectory
     */
    double createDirectPathPlan(const StateTriplet initialState,
                                StateTriplet targetState,
                                const float max_velocity = MAX_VELOCITY,
                                const double acceleration_phase_s = ACCELERATION_PHASE_SECS,
                                const double deceleration_phase_s = DECELERATION_PHASE_SECS,
                                const double angle_variation = 0);

    inline double normalizeYaw(double angle)
    {
      angle = fmod(angle + M_PI, 2 * M_PI);
      if (angle < 0)
        angle += 2 * M_PI;
      return angle - M_PI;
    }

    StateTriplet initialState;
    MSPTrajectory plan;

    float max_velocity;

    msp::MSPRapidTrajectoryGenerator segmentPlanner = msp::MSPRapidTrajectoryGenerator();
  };

} // namespace msp
