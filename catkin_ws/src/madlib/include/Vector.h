/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * 2D-Vector for points on track
  *
  * This file is part of Mini-Auto-Drive.
  *
  * Mini-Auto-Drive is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * Mini-Auto-Drive is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.
  *
  */

#ifndef _VECTOR_H
#define _VECTOR_H

#include <cmath>
#include <cstdint>

namespace modbas {

/**
 * Vector template class
 */
template<class T> class Vector
{
public:
  /**
   * @brief Vector
   * @param[in] s0 x-coordinate
   * @param[in] s1 y-coordinate
   */
  Vector(T s0, T s1) noexcept
    : s { s0, s1 }
  {
  }

  /**
   * @brief operator -= to subtract two vectors
   * @param[in] rhs Second vector
   * @return difference
   */
  Vector& operator-=(const Vector& rhs)
  {
    s[0] -= rhs.s[0];
    s[1] -= rhs.s[1];
    return *this;
  }

  /**
   * @brief operator - to subtract two vectors
   * @param[in] lhs First vector
   * @param[in] rhs Second vector
   * @return difference
   */
  friend Vector operator-(Vector lhs, const Vector& rhs)
  {
    lhs -= rhs;
    return lhs;
  }

  /**
   * @brief operator += to add two vectors
   * @param[in] rhs Second vector
   * @return difference
   */
  Vector& operator+=(const Vector& rhs)
  {
    s[0] += rhs.s[0];
    s[1] += rhs.s[1];
    return *this;
  }

  /**
   * @brief operator + to add two vectors
   * @param[in] lhs First vector
   * @param[in] rhs Second vector
   * @return difference
   */
  friend Vector operator+(Vector lhs, const Vector& rhs)
  {
    lhs += rhs;
    return lhs;
  }

  /**
   * @brief dist computes distance of two vectors
   * @param[in] rhs Second vector
   * @return distance
   */
  inline float dist(const Vector& rhs) const
  {
    Vector diff = rhs - *this;
    return diff.abs();
  }

  /**
   * @brief abs computes vector length
   * @return vector length
   */
  inline float abs() const
  {
    return std::sqrt(static_cast<float>(s[0])*static_cast<float>(s[0]) +
        static_cast<float>(s[1])*static_cast<float>(s[1]));
  }

  /**
   * @brief center computes center point between two positions (vectors)
   * @param[in] other Second vector
   * @param[out] c Center point
   */
  inline void center(const Vector& other, Vector<float>& c) const
  {
    c.s[0] = 0.5F * (static_cast<float>(s[0]) + static_cast<float>(other.s[0]));
    c.s[1] = 0.5F * (static_cast<float>(s[1]) + static_cast<float>(other.s[1]));
  }

  /**
   * @brief s The two coordinates of the vector
   */
  T s[2];
};

}

#endif // _VECTOR_H
