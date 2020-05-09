/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Straight Track Segment
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

#ifndef _STRAIGHT_SEGMENT_H
#define _STRAIGHT_SEGMENT_H

#include "TrackSegment.h"

namespace modbas {

/**
 * @brief The StraightSegment class to build tracks
 */
class StraightSegment : public TrackSegment
{
public:
  /**
   * @brief StraightSegment constructor
   * @param pose The pose at which the new segment is attached
   * @param[in] w Width of track segment [ m ]
   * @param[in] xe Arc length of track segment [ m ]
   */
  explicit StraightSegment(std::shared_ptr<TrackPose>& pose, const float w, const float xe, const RoadType type = RoadType::ONELANE) noexcept;

  /**
   * @brief StraightSegment copy constructor
   * @param[in] other The segment instance to be copied
   */
  StraightSegment(const StraightSegment& other) = delete;

  // lvalue assignment
  StraightSegment& operator=(const StraightSegment& other) = delete;

  /**
   * @brief samples segment by waypoints
   * @param[in] dxstart Delta to rear pose [ m ]
   * @param[in] dx Step size [ m ]
   * @param[in] alpha Position of line on track [0;1]
   * @param[out] spline Spline to be extended by waypoints
   * @return new dxstart for next segment
   */
  virtual float sample(const float dxstart, const float dx, const float alpha, Spline& spline);
};

}

#endif // _STRAIGHT_SEGMENT_H
