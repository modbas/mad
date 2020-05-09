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

#include "StraightSegment.h"
#include "Track.h"
#include "Utils.h"
#include <cmath>

namespace modbas {

StraightSegment::StraightSegment(std::shared_ptr<TrackPose>& pose,
                                 const float w,
                                 const float xe,
                                 const RoadType type) noexcept
    : TrackSegment { pose, w, type }
{
  this->xe = xe;
  frontPose = std::shared_ptr<TrackPose>(
        new TrackPose(pose->x + xe,
                      pose->s1 + xe * std::cos(pose->psi),
                      pose->s2 + xe * std::sin(pose->psi),
                      pose->psi));
  Track::singleton->addPose(frontPose);
  pose = frontPose;
  init();
}

float StraightSegment::sample(const float dxstart, const float dx, const float alpha, Spline& spline)
{
  std::shared_ptr<TrackPose> p = rearPose;
  const float cpsi { std::cos(p->psi) };
  const float spsi { std::sin(p->psi) };
  const float cpsi90 { std::cos(p->psi + Utils::pi * 0.5F) };
  const float spsi90 { std::sin(p->psi + Utils::pi * 0.5F) };
  float x { 0.0F };
  for (x = dxstart; x <= xe + Track::singleton->float_atol; x += dx) {
    spline.pushPoint(p->x + x,
                     std::array<float, Spline::dim>
                        { p->s1 + cpsi * x + (alpha - 0.5F) * w * cpsi90,
                          p->s2 + spsi * x + (alpha - 0.5F) * w * spsi90 },
                     id);
  }  
  return x - xe;
}

}
