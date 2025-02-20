#include <segment_trajectory_generator/SegmentedTrajectoryPlanner.h>
#include <utility>
#include <types/Vec3.h>

using namespace msp;

MSPTrajectory SegmentedTrajectoryPlanner::createOptimizedDirectPathPlan(const StateTriplet initialState,
                                                                        const StateTriplet targetState,
                                                                        const float max_velocity,
                                                                        const float deceleration_phase_secs)
{
  double dec_phase_secs = (deceleration_phase_secs == 0 || deceleration_phase_secs > DECELERATION_PHASE_SECS) ? deceleration_phase_secs : DECELERATION_PHASE_SECS;

  if (initialState.vel.GetNorm2() > MIN_VELOCITY)
  {
    // Optimizing the target heading
    const double heading_lower_bound = -M_PI_2;
    const double heading_upper_bound = M_PI_2;

    auto targetHeadingToOptimize = [&](double x)
    {
      return createDirectPathPlan(initialState, targetState, max_velocity, ACCELERATION_PHASE_SECS, dec_phase_secs, x);
    };
    std::pair<double, double> heading_result = boost::math::tools::brent_find_minima(
        targetHeadingToOptimize, heading_lower_bound, heading_upper_bound, std::numeric_limits<double>::digits10);

    plan.total_costs = createDirectPathPlan(initialState, targetState, max_velocity, ACCELERATION_PHASE_SECS,
                                            dec_phase_secs, heading_result.first);
  }
  else
    plan.total_costs =
        createDirectPathPlan(initialState, targetState, max_velocity, ACCELERATION_PHASE_SECS, dec_phase_secs, 0);

  return plan;
}

MSPTrajectory SegmentedTrajectoryPlanner::createCirclePathPlan(const StateTriplet initialState,
                                                               float max_velocity, float radius, int turns)
{
  // PlanItem item;

  // const int segment_count = 72;
  // const float segment_angle = M_PI * 2.0f / segment_count;
  // const float segment_length = segment_angle * radius;

  // const float omega = max_velocity / radius;

  // plan = MSPTrajectory();

  // // Todo: Consider constant acceleration

  // StateTriplet current_state = StateTriplet(initialState);

  // Vec3 vel1 = Vec3(0, max_velocity, 0);
  // Vec3 acc1 = Vec3(max_velocity * max_velocity / radius, 0, 0);
  // Vec3 nanv = Vec3();

  // if (initialState.vel.GetNorm2() < 0.01f)
  // {
  //   item = PlanItem(current_state);
  //   item.setTargetState(StateTriplet(nanv, nanv, acc1), PlanItem::TYPE_ACC, segment_length / max_velocity, max_velocity, true);
  //   plan.total_costs += segmentPlanner.generate(&item);
  //   plan.push(item);
  //   segmentPlanner.getSetpointAt(item.estimated_time_s, current_state);
  // }
  // else
  // {
  //   vel1.setTo(current_state.vel);
  //   vel1.scale(max_velocity / vel1.GetNorm2());
  //   acc1.setTo(current_state.vel);
  //   acc1.scale(omega * omega / (radius * acc1.GetNorm2()));
  //   vel1.rotateXY(segment_angle);
  //   acc1.rotateXY(M_PI / 2.0);
  // }

  // for (int i = 0; i < segment_count * turns; i++)
  // {
  //   item = PlanItem(current_state);
  //   item.setTargetState(StateTriplet(nanv, vel1, acc1), PlanItem::TYPE_VEL_ACC, segment_length / max_velocity, max_velocity, i == 0);
  //   plan.total_costs += segmentPlanner.generate(&item);
  //   current_state.set(item.targetState);
  //   // std::cout << current_state << "::" << current_state.acc.GetNorm2() << std::endl;
  //   plan.push(item);
  //   vel1.rotateXY(segment_angle);
  //   acc1.rotateXY(segment_angle);
  // }

  // return plan;
}

MSPTrajectory SegmentedTrajectoryPlanner::createPlanSegment(const StateTriplet initialState,
                                                            StateTriplet targetState,
                                                            const PlanItem::Type type = PlanItem::TYPE_POS_VEL,
                                                            const float max_velocity = MAX_VELOCITY)
{
  plan = MSPTrajectory();

  float duration = 0;
  if (targetState.pos.isFinite())
  {
    duration = (targetState.pos - initialState.pos).GetNorm2() * 2.0 / max_velocity;
  }
  else if (targetState.vel.isFinite())
  {
    double norm = targetState.vel.GetNorm2();
    if (norm > max_velocity)
      targetState.vel.scale(max_velocity / norm);
    duration = norm; // Assuming acceleration is 1 m/s2
  }
  else
    return plan;

  PlanItem item = PlanItem(initialState);
  item.setTargetState(targetState, type, duration);
  plan.total_costs = segmentPlanner.generate(&item);
  plan.push(item);

  return plan;
}

double SegmentedTrajectoryPlanner::createDirectPathPlan(const StateTriplet initialState,
                                                        const StateTriplet targetState,
                                                        const float max_vel, const double acceleration_phase_s,
                                                        const double deceleration_phase_secs,
                                                        const double angle_variation)
{
  StateTriplet s0;
  StateTriplet s1;
  StateTriplet s2;
  StateTriplet s3;

  float xy_time;

  double total_costs = 0;

  this->initialState = initialState;
  this->max_velocity = max_vel;

  double distance = (targetState.pos - initialState.pos).GetNorm2();
  double estimated_time = distance * 2.0f / max_velocity;

  plan = MSPTrajectory();

  if (estimated_time < (acceleration_phase_s + deceleration_phase_secs + MIN_CRUISE_TIME))
  {
    PlanItem simple = PlanItem(initialState, targetState, estimated_time * 2.0f);
    segmentPlanner.generate(&simple);
    plan.push(simple);
  }
  else
  {
    Vec3 vel1 = (targetState.pos - initialState.pos);
    double norm = vel1.GetNorm2();
    vel1 = vel1.scale(max_velocity / norm);

    if (abs(angle_variation - 0.05) > 0)
      vel1.rotateXY(angle_variation);

    double target_yaw = vel1.getXYAngle();
    double yaw_distance = std::abs(target_yaw - initialState.yaw.x);
    if (yaw_distance > M_PI)
    {
      target_yaw -= 2 * M_PI;
      yaw_distance = std::abs(target_yaw - initialState.yaw.x);
    }

    s0.set(initialState);

    // Add a turning item if required
    if (!std::isfinite(targetState.yaw.x) & initialState.vel.GetNorm2() < 0.2f & yaw_distance > M_PI_4)
    {
      auto yawTarget = targetState;
      yawTarget.yaw = {target_yaw, 0, 0};
      const double yaw_estimated_s = yaw_distance / MAX_YAWSPEED;
      PlanItem turn_phase = PlanItem(initialState, yawTarget, yaw_estimated_s, PlanItem::TYPE_YAW_ONLY);
      segmentPlanner.generate(&turn_phase);
      segmentPlanner.getSetpointAt(yaw_estimated_s, s0);
      plan.push(turn_phase);
    }

    // Add an acceleration phase
    PlanItem acceleration_phase = PlanItem(s0);
    acceleration_phase.setTargetState(StateTriplet(Vec3(), vel1, Vec3(0, 0, 0), targetState.yaw), PlanItem::TYPE_VEL_ACC,
                                      acceleration_phase_s, max_vel, true);
    segmentPlanner.generate(&acceleration_phase);
    segmentPlanner.getSetpointAt(acceleration_phase_s, s1);
    plan.push(acceleration_phase);

    // Add a cruising phase
    PlanItem cruise_phase = PlanItem(s1);
    if (deceleration_phase_secs > 0)
    {
      PlanItem tmp = PlanItem(targetState);
      tmp.setTargetState(StateTriplet(Vec3(), vel1, Vec3(0, 0, 0)), PlanItem::TYPE_VEL_ACC, -deceleration_phase_secs);
      total_costs += segmentPlanner.generate(&tmp);
      segmentPlanner.getSetpointAt(-deceleration_phase_secs, s2);

      xy_time = (s2.pos - s1.pos).GetNorm2() / s1.vel.GetNorm2();

      s2.yaw.setTo(targetState.yaw);
      cruise_phase.setTargetState(s2, PlanItem::TYPE_VEL, xy_time, max_velocity);
    }
    else
    {
      xy_time = (targetState.pos - s1.pos).GetNorm2() / s1.vel.GetNorm2();
      cruise_phase.setTargetState(StateTriplet(targetState.pos, vel1, Vec3(), targetState.yaw), PlanItem::TYPE_POS_VEL, xy_time,
                                  max_velocity);
    }

    total_costs += segmentPlanner.generate(&cruise_phase);
    plan.push(cruise_phase);

    // Add a breaking phase phase if rqeuested
    if (deceleration_phase_secs > 0)
    {
      segmentPlanner.getSetpointAt(xy_time, s3);

      PlanItem deceleration_phase = PlanItem(s3);
      deceleration_phase.setTargetState(targetState, PlanItem::TYPE_POS_VEL_ACC, deceleration_phase_secs);
      total_costs += segmentPlanner.generate(&deceleration_phase);
      plan.push(deceleration_phase);
    }
  }

  return total_costs;
}

MSPTrajectory SegmentedTrajectoryPlanner::getPlan(void)
{
  return plan;
}
