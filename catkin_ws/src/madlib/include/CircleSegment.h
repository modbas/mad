/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Circular Track Segment
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

#ifndef _CIRCLE_SEGMENT_H
#define _CIRCLE_SEGMENT_H

#include "TrackSegment.h"

namespace modbas {

/**
 * @brief The CircleSegment class to build tracks
 */
class CircleSegment : public TrackSegment
{
public:
  /**
   * @brief CircleSegment constructor
   * * @param pose The pose at which the new segment is attached
   * @param[in] w Width of track segment [ m ]
   * @param[in] r Radius of circle segment [ m ]
   * @param[in] rad Arc length of circle segment (positive for left turns, negativ for right turns) [ rad ]
   */
  explicit CircleSegment(std::shared_ptr<TrackPose>& pose, const float w, const float r,
                         const float rad, const RoadType type = RoadType::ONELANE) noexcept;

  /**
   * @brief CircleSegment copy constructor
   * @param[in] other The segment instance to be copied
   */
  CircleSegment(const CircleSegment& other) = delete;

  // lvalue assignment
  CircleSegment& operator=(const CircleSegment& other) = delete;

  /**
   * @brief samples segment by waypoints
   * @param[in] dxstart Delta to rear pose [ m ]
   * @param[in] dx Step size [ m ]
   * @param[in] alpha Position of line on track [0;1]
   * @param[out] spline Spline to be extended by waypoints
   * @return new dxstart for next segment
   */
  virtual float sample(const float dxstart, const float dx, const float alpha, Spline& spline);

private:
  const float r { 0.0F }; // radius [ m ]
  const float rad { 0.0F }; // arc length [ rad ], positive or negative

};

}


#endif // _CIRCLE_SEGMENT_H
