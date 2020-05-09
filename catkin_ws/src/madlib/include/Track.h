/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Track Definition
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

#ifndef _TRACK_H
#define _TRACK_H

#include <array>
#include <vector>
#include <cmath>
#include <memory>
#include <cstdint>
#include "TrackSegment.h"
#include "TrackPose.h"
#include "TrackCircuit.h"
#include "Utils.h"

namespace modbas {

/**
 * @brief The Track class represents the complete track aggregated from segments
 */
class Track
{
public:
  constexpr static float float_atol { 1e-3F }; /**< 1mm resolution to compared two points on track */

  /**
   * @brief singleton
   */
  static Track* singleton;

  /**
   * @brief Track constructor
   * @param[in] s1 width [ m ]
   * @param[in] s2 height [ m ]
   * @param[in] graphicsDx sampling step size for graphics [ m ]
   */
  explicit Track(const float s1, const float s2, float graphicsDx) noexcept;

  /**
   * @brief Add new segment to segment list
   * @param[in] segment New segment
   * @return false if there is no existing pose
   */
  bool addSegment(std::shared_ptr<TrackSegment> segment);

  /**
   * @brief Add new pose to pose list
   * @param[inout] pose New post or existing pose
   * @return false if pose is nullptr
   */
  bool addPose(std::shared_ptr<TrackPose>& pose);

  /**
   * @brief initCircuit Initializes new circuit in continuous segment-wise construction
   */
  void initCircuit();

  /**
   * @brief endLap Finalizes the new lap and adds it to lap list
   * @return false on error
   */
  bool finalizeCircuit();

  /**
   * @brief Read only access to segments
   * @return segments list
   */
  std::vector<std::shared_ptr<TrackSegment>>& getSegments()
  {
    return segments;
  }

  /**
     * @brief Sample computes splines. If segmentIds contains one id and this id references a circuit segment
     *        then a periodic spline of this circuit is computed.
     * @param[in] segmentsIds List of segment ids
     * @param[in] dx Sampling step size [ m ]
     * @param[in] alpha Denotes line on track, 0 for right border, 0.5 for center, 1 for left border [0;1]
     * @param[out] spline The spline interpolating the line on the track
     * @param[out] xEmd Arc length of track end (and not spline end)
     * @param[in] bc Boundary condition of spline
     * @return true on success
     */
  bool sample(std::vector<uint32_t>& segmentIds,
             const float dx,
             const float alpha,
             float& xEnd,
             Spline& spline);

  /**
   * @brief getNearestSegment returns id of segment which is closest to point s
   * @param[in] s point of car
   * @param[out] segmentId id of track segment
   * @param x arc length on track
   * @param dist the minimal distance
   * @return
   */
  bool getNearestSegment(const std::array<float, Spline::dim> s, uint32_t& segmentId, float& x, float& dist);

  /**
   * @brief getOccupancyGrid creates the occupancy grid of the track by sampling rectangles
   * @param[out] grid The grid
   * @param[out] smin Coordinates of lower left corner of grid [ m ]
   * @param[out] smax Coordinates of upper right corner of grid [ m ]
   * @param[out] size The grid dimensions [ m ]
   * @param[in] dx Sampling distance [ m ]
   * @param[in] curb Width of curb [ m ] (default: 0m)
   */
//  void getOccupancyGrid(std::vector<int8_t>& grid, std::array<float,Spline::dim>& smin, std::array<float,Spline::dim>& smax,
//                        std::array<uint32_t,Spline::dim>& size, const float dx, const float curb = 0.0F);

  const float graphicsDx; /**< sampling distance for waypoints for graphics */
  const float s1; /**< width */
  const float s2; /**< height */

private:    
  /**
   * @brief sample computes splines.
   * @param[in] segments Segment list
   * @param[in] dx Sampling step size [ m ]
   * @param[in] alpha Denotes line on track, 0 for right border, 0.5 for center, 1 for left border [0;1]
   * @param[out] spline The spline interpolating the line on the track
   * @param[out] xEmd Arc length of track end (and not spline end)
   * @param[in] bc Boundary condition of spline
   * @return true on success
   */
  bool sample(std::vector<std::shared_ptr<TrackSegment>>& segments,
             const float dx,
             const float alpha,
             Spline& spline,
             float& xEnd,
             const Spline::BoundaryCondition bc);

  std::vector<std::shared_ptr<TrackSegment>> segments; /**< list of track segments */
  std::vector<std::shared_ptr<TrackPose>> poses; /**< list of poses inbetween segments */
  std::shared_ptr<TrackSegment> firstLapSegment { nullptr }; /**< first segment of new lap */
  std::vector<std::shared_ptr<TrackCircuit>> circuits; /**< list of circular laps */
};

}

#endif // _TRACK_H
