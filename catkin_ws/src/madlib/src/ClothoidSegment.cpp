/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Clothoid Track Segment
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

#include "ClothoidSegment.h"
#include "Track.h"
#include "Utils.h"
#include <cmath>
#include <boost/bind.hpp>

namespace modbas {

ClothoidSegment::ClothoidSegment(std::shared_ptr<TrackPose>& pose, const float w, const float a,
                             const float rad, const ClothoidType clothoidType, const RoadType type) noexcept
  : TrackSegment { pose, w, type }, clothoidType { clothoidType }, a { a }, rad { rad }
{
  xe = std::sqrt(2.0F / a * std::fabs(rad)); // path length
  float dx { 1e-3F }; // step size approx 1mm
  const int n = static_cast<int>(std::floor(xe / dx + 0.5));
  dx = xe / static_cast<float>(n);
  StatesType s { { rearPose->s1, rearPose->s2, rearPose->psi } };
  boost::numeric::odeint::integrate_const(solver, boost::ref(*this), s, 0.0F, xe, dx);
  frontPose = std::shared_ptr<TrackPose>(
        new TrackPose(pose->x + xe,
                      s.at(0),
                      s.at(1),
                      pose->psi + rad));
  Track::singleton->addPose(frontPose);
  pose = frontPose;
  init();
}

float ClothoidSegment::sample(const float dxstart, const float dx, const float alpha, Spline& spline)
{
  float dxstep { 1e-3F }; // step size approx 1mm
  const int n = static_cast<int>(std::floor(dxstart / dxstep + 0.5));
  dxstep = dxstart / static_cast<float>(n);
  StatesType s { { rearPose->s1, rearPose->s2, rearPose->psi } };
  boost::numeric::odeint::integrate_const(solver, boost::ref(*this), s, 0.0F, dxstart, dxstep);
  const int32_t oversampling { 10 };
  boost::numeric::odeint::integrate_const(solver, boost::ref(*this), s, dxstart,
                                          xe + Track::singleton->float_atol,
                                          dx / static_cast<float>(oversampling),
                                          IntegrateObserver(rearPose, id, w, alpha, spline, oversampling));  
  return spline.breaks.back() + dx - frontPose->x;
}

void ClothoidSegment::operator()(const StatesType& x, StatesType& xd, const float t) noexcept
{
  const float b { (rad > 0.0F ) ? a : (-a) };
  if (clothoidType == ClothoidType::OPENING) {
    xd.at(0) = std::cos(x.at(2));
    xd.at(1) = std::sin(x.at(2));
    xd.at(2) = -b * (t - xe);
  } else {
    xd.at(0) = std::cos(x.at(2));
    xd.at(1) = std::sin(x.at(2));
    xd.at(2) = b * t;
  }
}


}
