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

#ifndef _CLOTHOID_SEGMENT_H
#define _CLOTHOID_SEGMENT_H

#include "TrackSegment.h"
#include <boost/numeric/odeint.hpp>
#include <array>

namespace modbas {


/**
 * @brief The ClothoidSegment class to build tracks
 */
class ClothoidSegment : public TrackSegment
{
public:
  enum class ClothoidType { OPENING, CLOSING }; /**< road type this lane belongs to */
  using StatesType = std::array<float, 3>;

  // Observer for fresnel integral
  struct IntegrateObserver
  {
      std::shared_ptr<TrackPose>& rearPose;
      const std::size_t id;
      const float w;
      const float alpha;
      Spline& spline;
      const int32_t oversampling;
      int32_t sample = 0;

      IntegrateObserver(std::shared_ptr<TrackPose>& rearPose, const std::size_t id,
                        const float w, const float alpha, Spline& spline,
                        const int32_t oversampling)
        : rearPose { rearPose }, id { id }, w { w }, alpha { alpha }, spline { spline },
          oversampling { oversampling }
          { }

      void operator()(const StatesType &x, const float t)
      {
        if (sample % oversampling == 0) {
          const float cpsi { std::cos(x.at(2)) };
          const float spsi { std::sin(x.at(2)) };
          spline.pushPoint(rearPose->x + t,
                           std::array<float, Spline::dim>
                              { x.at(0) - spsi * w * (alpha - 0.5F),
                                x.at(1) + cpsi * w * (alpha - 0.5F)},
                           id);
        }
        ++sample;
      }
  };


  /**
   * @brief ClothoidSegment constructor
   * * @param pose The pose at which the new segment is attached
   * @param[in] w Width of track segment [ m ]
   * @param[in] a Curvature gain (kappa = a*x)
   * @param[in] rad Arc length of Clothoid segment (positive for left turns, negativ for right turns) [ rad ]
   */
  explicit ClothoidSegment(std::shared_ptr<TrackPose>& pose, const float w, const float a,
                           const float rad, const ClothoidType clothoidType,
                           const RoadType type = RoadType::ONELANE) noexcept;

  /**
   * @brief ClothoidSegment copy constructor
   * @param[in] other The segment instance to be copied
   */
  ClothoidSegment(const ClothoidSegment& other) = delete;

  // lvalue assignment
  ClothoidSegment& operator=(const ClothoidSegment& other) = delete;

  // ODE for fresnel integral
  void operator()(const StatesType& x, StatesType& xd, const float t) noexcept;


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
  const ClothoidType clothoidType { ClothoidType::OPENING };
  const float a { 0.0F }; // curvature gain [ m ]
  const float rad { 0.0F }; // arc length [ rad ], positive or negative
  boost::numeric::odeint::runge_kutta4<StatesType> solver;
};

}


#endif // _Clothoid_SEGMENT_H
