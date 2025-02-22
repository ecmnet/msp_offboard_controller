#pragma once
#include <msp_offboard_controller/types/Vec3.h>
#include <iostream>
#include <array>


namespace msp {

//! State Triplet class
/*!
 * Contains a set of vectors representing position, velocity and acceleration
 */
class StateTriplet
{
public:
  Vec3 pos,vel,acc;
  Vec3 yaw;

StateTriplet(void):pos(),vel(),acc() {};
StateTriplet(Vec3 p):vel(0,0,0),acc(0,0,0) { pos = p; };
StateTriplet(Vec3 p, Vec3 v, Vec3 a):pos(p),vel(v),acc(a) {};
StateTriplet(Vec3 p, Vec3 v, Vec3 a, Vec3 w):pos(p),vel(v),acc(a),yaw(w) {};

StateTriplet(std::array<float, 3> p, std::array<float, 3> v, std::array<float, 3> a) {
    pos.x = p[0]; pos.y = p[1]; pos.z = p[2];
    vel.x = v[0]; vel.y = v[1]; vel.z = v[2];
    acc.x = a[0]; acc.y = a[1]; acc.z = a[2];
    yaw.setToNaN();
};

inline void set( StateTriplet t) { pos = t.pos; vel = t.vel; acc = t.acc; yaw = t.yaw; };

inline void set( Vec3 p, Vec3 v=Vec3(), Vec3 a=Vec3() ) { pos = p; vel = v; acc = a; }

inline void set( double px, double py, double pz, double vx=0, double vy=0, double vz=0, double ax=0, double ay=0, double az=0, double y = 0, double yr = 0)
{ pos.x = px; pos.y = py; pos.z = pz; vel.x = vx; vel.y = vy, vel.z = vz; acc.x = ax, acc.y = ay; acc.z = az; yaw.x = y; yaw.y = yr;};

inline void setToNaN(void) {  pos.setToNaN(); vel.setToNaN(); acc.setToNaN(); yaw.setToNaN(); };

inline void clear(void) { pos.clear(); vel.clear(); acc.clear(); yaw.clear(); }
    
};

inline std::ostream& operator<<(std::ostream& os, const StateTriplet& t) {
    os << "P" << t.pos << ", V" << t.vel << ", A" << t.acc << ", Y" << t.yaw;
    return os;
};

}

