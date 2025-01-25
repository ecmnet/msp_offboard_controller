#pragma once
#include <minimum_jerk_trajectories/RapidTrajectoryGenerator.h>
#include <types/PlanItem.h>

namespace msp
{

class MSPRapidTrajectoryGenerator : public minimum_jerk_trajectories::RapidTrajectoryGenerator
{
public:

  /*! Generates a single trajectory segment
   *
   * Calculate the full trajectory, for all the parameters defined so far.
   * @param item The PlanItem to be generated
   * @return returns the costs of the segment
   */
  float generate(PlanItem* item);

  /*! Gets a setpoint as StateTriplet for a certain point in time
   *
   * @param tf The relative time in seconds for the generated segment
   * @param triplet The returned setpoint triplet
   */
  void getSetpointAt(double tf, StateTriplet& triplet);

private:

  void set(PlanItem* item);

  const Vec3 gravity = Vec3(0, 0, -9.81);
};
}