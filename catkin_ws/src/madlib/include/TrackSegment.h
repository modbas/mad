/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Base class of track segments
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

#ifndef _TRACK_SEGMENT_H
#define _TRACK_SEGMENT_H

#include "TrackPose.h"
#include "TrackCircuit.h"
#include "Spline.h"
#include <memory>
#include <cstddef>

namespace modbas {

class TrackPose; /**< forward reference to TrackPose */

/**
 * @brief The TrackSegment base class of track segments
 */
class TrackSegment
{
public:
  enum class RoadType { ONELANE, PARKING }; /**< road type this lane belongs to */

  /**
   * @brief TrackSegment constructor
   * @param track The track to be extended
   * @param[in] w The segment width [ m ]
   */
  explicit TrackSegment(std::shared_ptr<TrackPose>& pose, const float w, const RoadType type = RoadType::ONELANE) noexcept;

  /**
   * @brief Copy constructor
   * @param[in] other The segment to be copied
   */
  TrackSegment(const TrackSegment& other) = delete;

  // lvalue assignment
  TrackSegment& operator=(const TrackSegment& other) = delete;

  /**
   * @brief samples segment by waypoints
   * @param[in] dxstart Delta to rear pose [ m ]
   * @param[in] dx Step size [ m ]
   * @param[in] alpha Position of line on track [0;1]
   * @param[out] spline Spline to be extended by waypoints
   * @return new dxstart for next segment
   */
  virtual float sample(const float dxstart, const float dx, const float alpha, Spline& spline) = 0;

  /**
   * @brief getStartX
   * @return arc length of segment start
   */
  float getStartX()
  {
    return rearPose->x;
  }

  /**
   * @brief getEndX
   * @return arc length of segment end
   */
  float getEndX()
  {
    return rearPose->x + xe;
  }

  const RoadType type { RoadType::ONELANE };
  std::size_t id; /**< id in segments list of Track */
  std::shared_ptr<TrackPose> rearPose; /**< pose at rear */
  std::shared_ptr<TrackPose> frontPose; /**< pose at front */
  std::shared_ptr<TrackSegment> leftSequment; /** sequment on the left side */
  std::shared_ptr<TrackSegment> rightSequment; /** sequment on the right side */
  Spline leftSpline; /**< left border spline (created by sample()) */
  Spline centerSpline; /**< center spline (created by sample()) */
  Spline rightSpline; /**< right spline (created by sample()) */
  std::shared_ptr<TrackCircuit> circuit { nullptr }; /**< circuit this segment may belong to */

protected:
  /**
   * @brief init
   */
  void init();

  const float w { 0.0F }; /**< width of segment [ m ] */
  float xe { 0.0F }; /**< path length [ m ], only positive */
  float graphicsDx { 0.0F }; /**< adapted sampling step size [ m ] */
};

}

#endif // _TRACK_SEGMENT_H
