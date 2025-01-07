#include <segment_trajectory_generator/SegmentedTrajectoryPlanner.h>
#include <utility>

using namespace msp;

std::queue<PlanItem> SegmentedTrajectoryPlanner::createOptimizedDirectPathPlan(const StateTriplet initialState,
                                                                               const Vec3 target_pos,
                                                                               const float max_velocity)
{
  if (initialState.vel.GetNorm2() > 0.1)
  {
    auto trajectoryToOptimize = [&](double x) { return createDirectPathPlan(initialState, target_pos, max_velocity, x); };

    const double lower_bound = -M_PI / 8.0;
    const double upper_bound = M_PI / 8.0;

    std::pair<double, double> result = boost::math::tools::brent_find_minima(
        trajectoryToOptimize, lower_bound, upper_bound, std::numeric_limits<double>::digits10);

    createDirectPathPlan(initialState, target_pos, max_velocity, result.first);
  }
  else
    createDirectPathPlan(initialState, target_pos, max_velocity);

  return plan;
}

double SegmentedTrajectoryPlanner::createDirectPathPlan(const StateTriplet initialState, const Vec3 target_pos,
                                                        const float max_vel, double angle_variation)
{
  StateTriplet s1;
  StateTriplet s2;
  StateTriplet s3;
  double total_costs = 0;

  this->initialState = initialState;
  this->max_velocity = max_vel;

  double distance = (target_pos - initialState.pos).GetNorm2();
  double estimated_time = distance * 2.0f / max_velocity;

  plan = std::queue<PlanItem>();

  if (estimated_time < (ACCELERATION_PHASE_SECS + DECELERATION_PHASE_SECS + 0.1f))
  {
    PlanItem simple = PlanItem(initialState, target_pos, estimated_time * 3.0f);
    segmentPlanner.generate(simple);
    plan.push(simple);
  }
  else
  {
    Vec3 vel1 = (target_pos - initialState.pos);
    double norm = vel1.GetNorm2();
    vel1 = vel1.scale(max_velocity / norm);

    if (abs(angle_variation - 0.005) > 0)
      vel1.rotateXY(angle_variation);

    PlanItem acceleration_phase = PlanItem(initialState);
    acceleration_phase.setTargetState(StateTriplet(Vec3(), vel1, Vec3(0, 0, 0)), PlanItem::TYPE_VEL_ACC,
                                      ACCELERATION_PHASE_SECS);
    segmentPlanner.generate(acceleration_phase);
    segmentPlanner.getSetpointAt(ACCELERATION_PHASE_SECS, s1);
    plan.push(acceleration_phase);

    PlanItem tmp = PlanItem(StateTriplet(target_pos, Vec3(0, 0, 0), Vec3(0, 0, 0)));
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
    deceleration_phase.setTargetState(StateTriplet(target_pos, Vec3(0, 0, 0), Vec3(0, 0, 0)),
                                      PlanItem::TYPE_POS_VEL_ACC, DECELERATION_PHASE_SECS);
    total_costs += segmentPlanner.generate(deceleration_phase);
    plan.push(deceleration_phase);
  }

  return total_costs;
}

std::queue<PlanItem> SegmentedTrajectoryPlanner::getPlan(void)
{
  return plan;
}
