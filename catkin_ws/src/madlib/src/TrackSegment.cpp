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

#include "Track.h"
#include "TrackSegment.h"

namespace modbas {

TrackSegment::TrackSegment(std::shared_ptr<TrackPose>& pose, const float w, const RoadType type) noexcept
  : type { type }, rearPose { pose }, w { w }
{
}

void TrackSegment::init()
{
  const int piecesCnt = static_cast<int>(std::floor(xe / Track::singleton->graphicsDx + 0.5F));
  graphicsDx = xe / static_cast<float>(piecesCnt);

  rightSpline.reserve(piecesCnt);
  centerSpline.reserve(piecesCnt);
  leftSpline.reserve(piecesCnt);
  sample(0.0F, graphicsDx, 0.0F, rightSpline);
  sample(0.0F, graphicsDx, 0.5F, centerSpline);
  // compute center spline coefficients for get_nearest service
  centerSpline.computeCoefficients(Spline::BoundaryCondition::natural);
  sample(0.0F, graphicsDx, 1.0F, leftSpline);
}

}
