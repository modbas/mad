/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Track
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
#include <cmath>
#include <cfloat>

namespace modbas {

Track* Track::singleton = nullptr;

Track::Track(const float s1, const float s2, const float graphicsDx) noexcept
  : graphicsDx { graphicsDx }, s1 { s1 }, s2 { s2 }
{
  singleton = this;
}


bool Track::addSegment(std::shared_ptr<TrackSegment> segment)
{
  bool ret = false;
  if (segment->rearPose != nullptr) {
    segment->id = segments.size();
    segments.push_back(segment);
    segment->rearPose->frontSegments.push_back(segment);
    segment->frontPose->rearSegments.push_back(segment);
    if (firstLapSegment == nullptr) {
      firstLapSegment = segment;
    }
    ret = true;
  }
  return ret;
}

bool Track::addPose(std::shared_ptr<TrackPose>& pose)
{
  bool ret = false;
  if (pose != nullptr) {
    // check if pose already exists
    std::shared_ptr<TrackPose> existingPose { nullptr };
    for (std::shared_ptr<TrackPose> p : poses) {
      // compare on equal physical poses
      if (*p == *pose) {
        existingPose = p;
        break;
      }
    }
    if (existingPose == nullptr) {
      pose->id = poses.size();
      poses.push_back(pose);
    } else {
      pose = existingPose;
    }
    ret = true;
  }
  return ret;
}

void Track::initCircuit()
{
  firstLapSegment = nullptr;
}

bool Track::finalizeCircuit()
{
  bool ret = false;
  if (firstLapSegment != nullptr) {
    // we have segments available
    std::shared_ptr<TrackCircuit> circuit = std::make_shared<TrackCircuit>();
    ret = circuit->init(circuit, firstLapSegment);
    if (ret) {
      circuit->id = circuits.size();
      circuits.push_back(circuit);
    }
    firstLapSegment = nullptr;
  }
  return ret;
}

bool Track::sample(std::vector<uint32_t>& segmentIds,
           const float dx,
           const float alpha,
           float& xEnd,
           Spline& spline)
{
  bool ret = false;
  std::shared_ptr<TrackCircuit> circuit { nullptr };
  if (segmentIds.size() == 1U
      && segmentIds.front() < segments.size()
      && (circuit = segments.at(segmentIds.front())->circuit) != nullptr) {
    // access circuit
    ret = sample(circuit->segments, dx, alpha, spline, xEnd, Spline::BoundaryCondition::periodic);
  } else {
    // no circuit, subsequence of segments
    std::vector<std::shared_ptr<TrackSegment>> segs;
    for (uint32_t id : segmentIds) {
      if (id < segments.size()) {
        segs.push_back(segments.at(id));
        ret = true;
      } else {
        ret = false;
        break;
      }
    }
    if (ret) {
      ret = sample(segs, dx, alpha, spline, xEnd, Spline::BoundaryCondition::natural);
    }
  }
  return ret;
}

bool Track::getNearestSegment(const std::array<float, Spline::dim> s, uint32_t& segmentId, float& x, float& dist)
{
  bool ret { false };
  dist = FLT_MAX;
  for (auto& segment : segments) {
    float segd { 0.0F };
    float segx { 0.0F };
    segment->centerSpline.getNearest(s, segx, segd);
    if (segd < dist) {
      dist = segd;
      segmentId = segment->id;
      x = segx;
      ret = true;
    }
  }
  return ret;
}

bool Track::sample(std::vector<std::shared_ptr<TrackSegment>>& segments,
                   const float dx,
                   const float alpha,
                   Spline& spline,
                   float& xEnd,
                   const Spline::BoundaryCondition bc)
{
  bool ret = true;
  float splineDx = dx;
  if (bc == Spline::BoundaryCondition::periodic) {
    // adapt sampling step size
    const float xe = segments.back()->getEndX() - segments.front()->getStartX();
    const int piecesCnt = static_cast<int>(std::floor(xe / dx + 0.5));
    splineDx = xe / static_cast<float>(piecesCnt);
    spline.reserve(piecesCnt);
  }
  float dxstart { 0.0F };
  for (auto& segment : segments) {
      dxstart = segment->sample(dxstart, splineDx, alpha, spline);
  }
  // ensure monotonic breaks (in case of segment sequence extension accross circuit boundary
  for (int32_t idx = spline.breaks.size()-2; idx >= 0; --idx)
  {
    spline.breaks.at(idx) = spline.breaks.at(idx+1) - splineDx;
  }
  spline.computeCoefficients(bc);
  xEnd = spline.breaks.back() + dx - dxstart;
  return ret;
}

//void Track::getOccupancyGrid(std::vector<int8_t>& grid,
//                             std::array<float,Spline::dim>& smin, std::array<float,Spline::dim>& smax,
//                             std::array<uint32_t,Spline::dim>& size, const float dx, const float curb)
//{
//    // grid boundaries
//    for (int j = 0; j < Spline::dim; ++j) {
//        smin[j] = FLT_MAX;
//        smax[j] = -FLT_MAX;
//    }
//    for (int i = 0; i < static_cast<int>(centerSpline.breaks.size()); ++i) {
//        for (int j = 0; j < Spline::dim; ++j) {
//            if (rightSpline.vals[i][j] < smin[j]) {
//                smin[j] = rightSpline.vals[i][j];
//            }
//            if (rightSpline.vals[i][j] > smax[j]) {
//                smax[j] = rightSpline.vals[i][j];
//            }
//            if (leftSpline.vals[i][j] < smin[j]) {
//                smin[j] = leftSpline.vals[i][j];
//            }
//            if (leftSpline.vals[i][j] > smax[j]) {
//                smax[j] = leftSpline.vals[i][j];
//            }
//        }
//    }
//    for (int j = 0; j < Spline::dim; ++j) {
//        int32_t imin = static_cast<int32_t>(std::floor(smin[j] / dx)) - 1;
//        int32_t imax = static_cast<int32_t>(std::ceil(smax[j] / dx)) + 1;
//        smin[j] = static_cast<float>(imin) * dx;
//        smax[j] = static_cast<float>(imax) * dx;;
//        size[j] = imax - imin;
//    }

//    // all grid elements are occupied by default
//    grid.resize(size[0]*size[1]);
//    for (auto& elem : grid) {
//        elem = 100;
//    }

//    // loop over track and free grid elements
//    float dx2 { dx * 0.25F }; // oversample
//    for (float x = 0.0F; x <= centerSpline.breaks.back(); x += dx2) {
//        std::array<float,2> right;
//        std::array<float,2> rightd;
//        std::array<float,2> rightdd;
//        rightSpline.interpolate(x, right, rightd, rightdd);
//        std::array<float,2> left;
//        std::array<float,2> leftd;
//        std::array<float,2> leftdd;
//        leftSpline.interpolate(x, left, leftd, leftdd);
//        float dist { std::sqrt(std::pow(left[0]-right[0], 2.0F) + std::pow(left[1]-right[1], 2.0F)) };
//        for (float w = (curb+dx+float_atol) / dist; w <= 1.0F - (curb+dx+float_atol) / dist; w += dx2/dist) {
//            std::array<float,2> s { { right[0] + (left[0]-right[0]) * w , right[1] + (left[1]-right[1]) * w } };
//            uint32_t i { static_cast<uint32_t>(std::floor((s[1]-smin[1]) / dx)) * size[0]
//                        + static_cast<uint32_t>(std::floor((s[0]-smin[0]) / dx)) };
//            grid.at(i) = 0;
//        }
//    }
//}

}
