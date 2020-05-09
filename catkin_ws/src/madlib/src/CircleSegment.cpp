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

#include "CircleSegment.h"
#include "Track.h"
#include "Utils.h"
#include <cmath>

namespace modbas {

CircleSegment::CircleSegment(std::shared_ptr<TrackPose>& pose, const float w, const float r,
                             const float rad, const RoadType type) noexcept
  : TrackSegment { pose, w, type }, r { r }, rad { rad }
{
  xe = std::fabs(rad) * r; // path length
  const float sign = (rad < 0.0F) ? -1.0F : 1.0F;
  const float center1 = pose->s1 + sign * r * std::cos(pose->psi + Utils::pi * 0.5F);
  const float center2 = pose->s2 + sign * r * std::sin(pose->psi + Utils::pi * 0.5F);
  frontPose = std::shared_ptr<TrackPose>(
        new TrackPose(pose->x + xe,
                      center1 - sign * r * std::cos(pose->psi + Utils::pi * 0.5F + rad),
                      center2 - sign * r * std::sin(pose->psi + Utils::pi * 0.5F + rad),
                      pose->psi + rad));
  Track::singleton->addPose(frontPose);
  pose = frontPose;
  init();
}

float CircleSegment::sample(const float dxstart, const float dx, const float alpha, Spline& spline)
{
  std::shared_ptr<TrackPose> p = rearPose;
  const float sign = (rad < 0.0F) ? -1.0F : 1.0F;
  const float center1 = p->s1 - sign * r * std::cos(p->psi - Utils::pi * 0.5F);
  const float center2 = p->s2 - sign * r * std::sin(p->psi - Utils::pi * 0.5F);
  const float radius = r - sign * (alpha - 0.5F) * w;
  float x { 0.0F };
  for (x = dxstart; x <= xe + Track::singleton->float_atol; x += dx) {
    spline.pushPoint(p->x + x,
                     std::array<float, Spline::dim>
                        { center1 + sign * radius * std::cos(p->psi - Utils::pi * 0.5F + sign * x / r),
                          center2 + sign * radius * std::sin(p->psi - Utils::pi * 0.5F + sign * x / r) },
                     id);
  }
  return x - xe;
}

}
