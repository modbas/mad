/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Cubic spline to interpolate reference paths
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

#ifndef _SPLINE_H
#define _SPLINE_H

#include <vector>
#include <array>
#include <cmath>
#include "TridiagonalSystem.h"

namespace modbas {

/**
 * @brief The Spline class for cubic splines
 */
class Spline
{
public:
  constexpr static const int dim = 2; /**< spline dimensions on planar surface */
  constexpr static const int order = 3; /**< cubic spline */

  enum class BoundaryCondition { natural, periodic }; /**< boundary conditions of cubic spline */

  /**
   * @brief Spline constructor
   */
  explicit Spline() noexcept;

  /**
   * @brief Spline constructor interpolates waypoints
   * @param[in] breaks List of arc lengths of waypoints [ m ]
   * @param[in] vals0 List of x-coordinates of waypoints [ m ]
   * @param[in] vals1 List of y-coordinates of waypoints [ m ]
   * @param[in] segmentIds Id of segment at every waypoint
   */
  explicit Spline(const std::vector<float>& breaks,
                  const std::vector<float>& vals0,
                  const std::vector<float>& vals1,
                  const std::vector<uint32_t>& segmentIds) noexcept;

  /**
   * @brief pushPoint adds one waypoint to internal list of waypoints
   * @param[in] brk Arc length of waypoint [ m ]
   * @param[in] val Coordinates of waypoint [ m ]
   * @param[in] segmentId Segment id of waypoint
   */
  void pushPoint(const float brk, const std::array<float, dim>& val,
                 const uint32_t segmentId = 0U);

  /**
   * @brief reserve preallocates memory for waypoints
   * @param[in] pieces_cnt expected number of waypoints
   */
  void reserve(const int pieces_cnt);

  /**
   * @brief computeCoefficients creates the spline. To be called after creating all waypoints.
   * @param[in] bc Boundary conditions
   */
  void computeCoefficients(const BoundaryCondition bc = BoundaryCondition::periodic);

  /**
   * @brief interpolate interpolates on the spline
   * @param[in] x The arc length of the point to be interpolated [ m ]
   * @param[out] y The coordinates of the interpolated point [ m ]
   * @param[out] yd The first derivative of the coordinates (tangential vector) [ 1 ]
   * @param[out] ydd The second derivative of the coordinates (normal vector) [ 1/m ]
   * @param[in] pieceIdx Optional waypoint index of interval on spline to speed up interpolation (default -1: search for interval on spline)
   */
  void interpolate(float x, std::array<float, dim>& y,
                   std::array<float, dim>& yd, std::array<float, dim>& ydd,
                   const int pieceIdx = -1);

  /**
   * @brief getNearest returns nearest point on spline (point which has minimal distance to y)
   * @param[in] y Coordinates of point next to spline (e.g., car position) [ m ]
   * @param[out] x Arc length of nearest point on spline [ m ]
   * @param[out] dist Approximate distance to spline. Do not use for control functions! [ m ]
   * @return Waypoint index of corresponding spline interval
   */
  int getNearest(const std::array<float, dim>& y, float& x, float& dist);

  /**
   * @brief getValVector extract one coordinate of the waypoints
   * @param val List of waypoints values of this one coordinate [ m ]
   * @param idx The coordinate index, may be 0 or 1 for planar splines
   */
  void getValVector(std::vector<float>& val, int32_t idx);

  std::vector<float> breaks; /**< internal arc lengths of waypoints [ m ] */
  std::vector<std::array<float, dim>> vals; /**< internal coordinates of waypoints [ m ] */
  std::vector<std::array<std::array<float,4>, dim>> coefs; /**< internal polynomial coefficients of spline intervals */
  std::vector<uint32_t> segmentIds; /**< segment id of each break */
  BoundaryCondition boundaryCondition { BoundaryCondition::natural }; /**< boundary conditions of spline */


private:
  /**
   * @brief binarySearch searches for spline interval (called from interpolate)
   * @param[in] x Arc length of interpolated point
   * @return Waypoint index of spline interval
   */
  int binarySearch(const float x);

  /**
   * @brief getDistance Computes distance of point y to waypoint with index i
   * @param[in] y Point coordinates [ m ]
   * @param[in] i Waypoint index
   * @return distance [ m ]
   */
  inline float getDistance(const std::array<float, dim>& y, const int i)
  {
    const std::array<float, dim> yd { { y.at(0)-vals.at(i).at(0), y.at(1)-vals.at(i).at(1) } };
    return std::sqrt(yd.at(0)*yd.at(0) + yd.at(1)*yd.at(1));
  }

  /**
   * @brief computeCoefficientsNatural Computes spline coefficient in case of natural boundary conditions
   */
  void computeCoefficientsNatural();

  /**
   * @brief computeCoefficientsNatural Computes spline coefficient in case of periodic boundary conditions (circular tracks)
   */
  void computeCoefficientsPeriodic();
};

}

#endif // _SPLINE_H
