/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <assert.h>
#include <math.h>
#include <limits>
#include <ostream>

//! 3D vector class
/*!
 * A 3D vector class, to make passing arguments easier, and allow easy addition etc. of vectors.
 * Could be readily replaced with arrays of doubles, but this would complicate some of the code.
 */
class Vec3
{
public:
  double x, y, z;  //!< the three components of the vector

  Vec3(void)
    : x(std::numeric_limits<double>::quiet_NaN())
    , y(std::numeric_limits<double>::quiet_NaN())
    , z(std::numeric_limits<double>::quiet_NaN()){};                    //!< Initialises all members to NaN
  Vec3(double xin, double yin, double zin) : x(xin), y(yin), z(zin){};  //!< Initialise vector

  //! Getter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline double operator[](int i) const
  {
    switch (i)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    // we're doing something wrong if we get here
    assert(0);
    return std::numeric_limits<double>::quiet_NaN();
  }

  //! Setter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline double& operator[](int i)
  {
    switch (i)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    // we're doing something wrong if we get here
    assert(0);
    // fail loudly:
    x = y = z = std::numeric_limits<double>::quiet_NaN();
    return x;
  }

  inline void setToNaN(void) {
     x = std::numeric_limits<double>::quiet_NaN();
     y = std::numeric_limits<double>::quiet_NaN();
     z = std::numeric_limits<double>::quiet_NaN();
  }

  inline void clear(void) {
     x = 0; y = 0; z = 0;
  }


  //! Calculate the xydot product of two vectors
  inline double DotXY(const Vec3 rhs) const
  {
    return x * rhs.x + y * rhs.y;
  };

  //! Calculate the Euclidean norm of the vector, squared (= sum of squared elements).
  inline double GetNorm2SquaredXY(void) const
  {
    return this->DotXY(*this);
  };

  //! Calculate the Euclidean norm of the vector.
  inline double GetNorm2XY(void) const
  {
    return sqrt(GetNorm2SquaredXY());
  };

  inline double getXYAngle(const Vec3 rhs)
  {
    return getXYAngle(x - rhs.x, y - rhs.y);
  }

  inline double getXYAngle()
  {
    return getXYAngle(x, y);
  }

  inline double getXYAngle(const float dx, const float dy)
  {
    if (dx == 0 && dy >= 0)
      return M_PI_2;
    if (dx == 0 && dy < 0)
      return -M_PI_2;
    if ((dx >= 0 && dy > 0) || (dx >= 0 && dy < 0))
      return atan(dy / dx);
    if ((dx < 0 && dy >= 0) || (dx < 0 && dy < 0))
      return (atan(dy / dx) + M_PI);
    return 0;
  }

  //   inline double getXYAngleMin(const Vec3 rhs) {
  //      double a = getXYAngle(rhs);
  // 	 return a > M_PI ? - (M_PI - a ) : a;
  //   }

  inline Vec3 scale(const float scale) {
    return Vec3( x * scale, y * scale, z * scale);
  }

  //! Calculate the dot product of two vectors
  inline double Dot(const Vec3 rhs) const
  {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  };

  //! Calculate the cross product of two vectors
  inline Vec3 Cross(const Vec3 rhs) const
  {
    return Vec3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
  };

  //! Calculate the Euclidean norm of the vector, squared (= sum of squared elements).
  inline double GetNorm2Squared(void) const
  {
    return this->Dot(*this);
  };

  //! Calculate the Euclidean norm of the vector.
  inline double GetNorm2(void) const
  {
    return sqrt(GetNorm2Squared());
  };

  //! Get the unit vector pointing along the same direction as this vector. Will fail for zero vectors.
  Vec3 GetUnitVector(void) const
  {
    return (*this) / this->GetNorm2();
  };

  inline Vec3 operator+(const Vec3 rhs) const
  {
    return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
  };
  inline Vec3 operator-(const Vec3 rhs) const
  {
    return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
  };
  inline Vec3 operator/(const double rhs) const
  {
    return Vec3(x / rhs, y / rhs, z / rhs);
  };

  inline void setFromAngle(double azimuth, double polar, double scalar=1.0) 
  {

   x = std::sin(polar) * std::cos(azimuth) * scalar;
   y = std::sin(polar) * std::sin(azimuth) * scalar;
   z = std::cos(polar) * scalar;
  };

  inline void rotateXY(double angle) {
  
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    double x_original = x;
    double y_original = y;

    x = x_original  * cosA - y_original * sinA;
    y = x_original  * sinA + y_original * cosA;
    
    };

}; 

// Some operator overloading:
inline Vec3 operator*(const double lhs, const Vec3 rhs)
{
  return Vec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
};
inline Vec3 operator*(const Vec3 lhs, const double rhs)
{
  return Vec3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
};
inline Vec3 operator/(const Vec3 lhs, const double rhs)
{
  return Vec3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
};

inline std::ostream& operator<<(std::ostream& os, const Vec3& t) {
    os << "(" << t.x << "," << t.y << "," << t.z << ")";
    return os;
};





