#include <segment_trajectory_generator/SegmentedTrajectoryPlanner.h>
#include <utility>
#include <types/Vec3.h>

using namespace msp;

MSPTrajectory SegmentedTrajectoryPlanner::createOptimizedDirectPathPlan(const StateTriplet initialState,
                                                                               const StateTriplet targetState,
                                                                               const float max_velocity,
                                                                               const float acc_phase_secs)
{
  if (initialState.vel.GetNorm2() > 0.1)
  {
    // Optimizing the target heading
    const double heading_lower_bound = -M_PI / 8.0;
    const double heading_upper_bound = M_PI / 8.0;

    auto targetHeadingToOptimize = [&](double x) {
      return createDirectPathPlan(initialState, targetState, max_velocity, acc_phase_secs, x);
    };
    std::pair<double, double> heading_result = boost::math::tools::brent_find_minima(
        targetHeadingToOptimize, heading_lower_bound, heading_upper_bound, std::numeric_limits<double>::digits10);

    createDirectPathPlan(initialState, targetState, max_velocity, acc_phase_secs, heading_result.first);
  }
  else
    createDirectPathPlan(initialState, targetState, max_velocity, acc_phase_secs, 0);

  return plan;
}

MSPTrajectory SegmentedTrajectoryPlanner::createCirclePathPlan(const StateTriplet initialState, Vec3 center, float max_velocity, float radius, int turns ) {

    const int   segment_count  = 32;
    const float segment_angle  = M_PI * 2.0f / segment_count;
    const float segment_length = segment_angle * radius;

    plan = MSPTrajectory();

    //Todo: Plan path to the tangent of the circle starting from the center (initialState = center)

    StateTriplet current_state = StateTriplet(initialState);
    
    Vec3 vel1 = Vec3(0,max_velocity,0);

      PlanItem item = PlanItem(current_state);
      item.setTargetState(StateTriplet(Vec3(),vel1,Vec3()),PlanItem::TYPE_VEL, 2.0 * segment_length / max_velocity);
      segmentPlanner.generate(item);
      plan.push(item);
      segmentPlanner.getSetpointAt(2.0 * segment_length / max_velocity,current_state);

    for(int i=0;i<segment_count*turns;i++) {
      item = PlanItem(current_state);
      item.setTargetState(StateTriplet(Vec3(),vel1,Vec3()),PlanItem::TYPE_VEL, segment_length / max_velocity);
      segmentPlanner.generate(item);
      plan.push(item);
      segmentPlanner.getSetpointAt(segment_length / max_velocity,current_state);
      vel1.rotateXY(segment_angle);
    }

    return plan;

}

double SegmentedTrajectoryPlanner::createDirectPathPlan(const StateTriplet initialState, const StateTriplet targetState,
                                                        const float max_vel,
                                                        const double acceleration_phase_s,
                                                        const double angle_variation)
{
  StateTriplet s1;
  StateTriplet s2;
  StateTriplet s3;
  double total_costs = 0;

  this->initialState = initialState;
  this->max_velocity = max_vel;

  double distance = (targetState.pos - initialState.pos).GetNorm2();
  double estimated_time = distance * 2.0f / max_velocity;

  plan = MSPTrajectory();

  if (estimated_time < (acceleration_phase_s + DECELERATION_PHASE_SECS + 0.1f))
  {
    PlanItem simple = PlanItem(initialState, targetState, estimated_time * 3.0f);
    segmentPlanner.generate(simple);
    plan.push(simple);
  }
  else
  {
    Vec3 vel1 = (targetState.pos - initialState.pos);
    double norm = vel1.GetNorm2();
    vel1 = vel1.scale(max_velocity / norm);

    if (abs(angle_variation - 0.005) > 0)
      vel1.rotateXY(angle_variation);

    PlanItem acceleration_phase = PlanItem(initialState);
    acceleration_phase.setTargetState(StateTriplet(Vec3(), vel1, Vec3(0, 0, 0)), PlanItem::TYPE_VEL_ACC,
                                      acceleration_phase_s);
    segmentPlanner.generate(acceleration_phase);
    segmentPlanner.getSetpointAt(acceleration_phase_s, s1);
    plan.push(acceleration_phase);

    PlanItem tmp = PlanItem(targetState);
    tmp.setTargetState(StateTriplet(Vec3(), vel1, Vec3(0, 0, 0)), PlanItem::TYPE_VEL_ACC, -DECELERATION_PHASE_SECS);
    total_costs += segmentPlanner.generate(tmp);
    segmentPlanner.getSetpointAt(-DECELERATION_PHASE_SECS, s2);

    float xy_time = (s2.pos - s1.pos).GetNorm2() / max_velocity;

    PlanItem cruise_phase = PlanItem(s1);
    cruise_phase.setTargetState(s2, PlanItem::TYPE_VEL, xy_time, max_velocity);
    total_costs += segmentPlanner.generate(cruise_phase);
    segmentPlanner.getSetpointAt(xy_time, s3);
    plan.push(cruise_phase);

    PlanItem deceleration_phase = PlanItem(s3);
    deceleration_phase.setTargetState(targetState,
                                      PlanItem::TYPE_POS_VEL_ACC, DECELERATION_PHASE_SECS);
    total_costs += segmentPlanner.generate(deceleration_phase);
    plan.push(deceleration_phase);
  }

  return total_costs;
}

MSPTrajectory SegmentedTrajectoryPlanner::getPlan(void)
{
  return plan;
}
