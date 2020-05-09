/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Circular Track
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

#ifndef _TRACK_CIRCUIT_H
#define _TRACK_CIRCUIT_H

#include "TrackSegment.h"
#include "Spline.h"
#include <memory>
#include <vector>

namespace modbas {

class TrackSegment; /**< forward reference to TrackSegment */
class TrackPose; /**< forward reference to TrackPose */

/**
 * @brief The TrackCircuit represents laps in tracks
 */
class TrackCircuit
{
public:
  /**
   * @brief TrackLap constructor
   * @param[in] startSegment The first segment of new lap
   * @param[in] endSegment The last segment of new lap
   */
  TrackCircuit() noexcept;

  /**
   * @brief init
   * @param[in] circuit shared_ptr to self
   * @param[in] startSegment
   * @param[out] endSegment
   * @return true on success
   */
  bool init(std::shared_ptr<TrackCircuit> circuit, std::shared_ptr<TrackSegment> startSegment);

  /**
   * @brief Copy constructor
   * @param[in] other The lap to be copied
   */
  TrackCircuit(const TrackSegment& other) = delete;

  // lvalue assignment
  TrackCircuit& operator=(const TrackSegment& other) = delete;

  std::vector<std::shared_ptr<TrackSegment>> segments; /**< segment sequence of circuit */
  std::size_t id; /**< id in circuit list of Track */
};

}

#endif // _TRACK_LAP_H
