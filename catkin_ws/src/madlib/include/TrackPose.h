/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Track pose representing spline waypoints
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

#ifndef _TRACK_POSE_H
#define _TRACK_POSE_H

#include "Track.h"
#include "Utils.h"
#include <cstddef>
#include <cmath>

namespace modbas {

class TrackSegment; /**< forward reference to TrackSegment */

/**
 * @brief The TrackPose struct
 */
struct TrackPose {
  /**
   * @brief TrackPose default constructor
   */
  TrackPose() {}

  /**
   * @brief TrackPose constructor
   * @param[in] x Arc length [ m ]
   * @param[in] s1 x-coordinate [ m ]
   * @param[in] s2 y-coordinate [ m ]
   * @param[in] psi orientation (yaw angle) [ rad ]
   */
  TrackPose(const float x, const float s1, const float s2, const float psi)
    : x(x), s1(s1), s2(s2), psi(psi) {}

  /**
   * @brief Move contructor
   * @param other Object to be moved
   */
  TrackPose(TrackPose&& other)
    : id(other.id), x(other.x), s1(other.s1), s2(other.s2), psi(other.psi) {}

  // lvalue assignment
  TrackPose& operator=(const TrackPose& other) = default;


  /**
    @brief Compare on equal
    */
  bool operator==(const TrackPose& other)
  {
    const float tol = 1e-3F;
    return std::fabs(s1 - other.s1) <= tol
           && std::fabs(s2 - other.s2) <= tol
           && Utils::compareRads(psi, other.psi, tol);
  }

  /**
    @brief Compare on unequal
    */
  bool operator!=(const TrackPose& other)
  {
    return (*this == other) == false;
  }


  std::size_t id; /** < idx in pose list of Track **/
  float x; /**< arc length [ m ] */
  float s1; /**< x-coordinate [ m ] */
  float s2; /**< y-coordinate [ m ] */
  float psi; /**< orientation (yaw angle) [ rad ] */
  std::vector<std::shared_ptr<TrackSegment>> rearSegments;
  std::vector<std::shared_ptr<TrackSegment>> frontSegments;
};

}

#endif // _TRACK_POSE_H
