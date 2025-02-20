#include <segment_trajectory_generator/MSPRapidTrajectoryGenerator.h>

using namespace msp;

float MSPRapidTrajectoryGenerator::generate(PlanItem *item)
{
  this->set(item);
  this->Generate(item->estimated_time_s);
  for (int i = 0; i < 3; i++)
  {
    item->alpha[i] = this->GetAxisParamAlpha(i);
    item->beta[i] = this->GetAxisParamBeta(i);
    item->gamma[i] = this->GetAxisParamGamma(i);
  }
  this->yaw.GenerateTrajectory(item->estimated_time_s);
  item->costs = this->GetCost();
  return item->costs;
}

void MSPRapidTrajectoryGenerator::getSetpointAt(double tf, StateTriplet &triplet)
{
  triplet.pos.setTo(this->GetPosition(tf));
  triplet.vel.setTo(this->GetVelocity(tf));
  triplet.acc.setTo(this->GetAcceleration(tf));
  triplet.yaw.x = this->yaw.GetPosition(tf);
  triplet.yaw.y = this->yaw.GetVelocity(tf);
}

void MSPRapidTrajectoryGenerator::set(PlanItem *item)
{
  this->setInitialState(item->initialState.pos, item->initialState.vel, item->initialState.acc, gravity);

  switch (item->planning_type)
  {
  case PlanItem::TYPE_POS_VEL_ACC:
    this->SetGoalPosition(item->targetState.pos);
    this->SetGoalVelocity(item->targetState.vel);
    this->SetGoalAcceleration(item->targetState.acc);
    break;
  case PlanItem::TYPE_POS_VEL:
    this->SetGoalPosition(item->targetState.pos);
    this->SetGoalVelocity(item->targetState.vel);
    break;
  case PlanItem::TYPE_POS:
    this->SetGoalPosition(item->targetState.pos);
    break;
  case PlanItem::TYPE_VEL_ACC:
    this->SetGoalVelocity(item->targetState.vel);
    this->SetGoalAcceleration(item->targetState.acc);
    break;
  case PlanItem::TYPE_VEL:
    this->SetGoalVelocity(item->targetState.vel);
    break;
  case PlanItem::TYPE_ACC:
    this->SetGoalAcceleration(item->targetState.acc);
    break;
  case PlanItem::TYPE_YAW_ONLY:
    break;
  }

  this->yaw.Reset();
  this->yaw.SetInitialState(item->initialState.yaw.x, item->initialState.yaw.y, 0);
  this->yaw.SetGoalPosition(item->targetState.yaw.x);
  this->yaw.SetGoalVelocity(item->targetState.yaw.y);
  this->yaw.SetGoalAcceleration(0);
}
