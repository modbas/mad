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

#include "Track.h"
#include "TrackCircuit.h"
#include <cstdint>
#include <ros/ros.h>

namespace modbas {

TrackCircuit::TrackCircuit() noexcept
{
}

bool TrackCircuit::init(std::shared_ptr<TrackCircuit> circuit, std::shared_ptr<TrackSegment> startSegment)
{
  bool ret = false;
  const uint32_t cntMax = 1000U; // avoid endless loops

  std::shared_ptr<TrackSegment> segment = startSegment;
  for (uint32_t cnt = 0U; segment != nullptr && cnt < cntMax; ++cnt) {
    segments.push_back(segment);
    if (segment->frontPose->frontSegments.empty()) {
      // we have no loop
      segment = nullptr;
      ROS_ERROR("track has dead end");
    } else {
      // next segment
      segment = segment->frontPose->frontSegments.front();
      if (segment == startSegment) {
        segment = nullptr;
        ret = true;
      }
    }
  }
  if (ret) {
    for (std::shared_ptr<TrackSegment> segment : segments) {
      segment->circuit = circuit;
    }
  } else {
    ROS_ERROR("first segment is not in a loop");
  }

  return ret;
}

}
