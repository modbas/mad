/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Track ROS Node
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

#ifndef TRACK_NODE_H
#define TRACK_NODE_H

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdio>
#include "Track.h"
#include "StraightSegment.h"
#include "CircleSegment.h"
#include "ClothoidSegment.h"
#include "Utils.h"
#include "madmsgs/TrackGetGraph.h"
#include "madmsgs/TrackGetNearest.h"
#include "madmsgs/TrackGetWaypoints.h"

using namespace std;

namespace modbas {

/**
 * @brief The TrackNode class
 */
class TrackNode
{

public:

  /**
   * @brief TrackNode
   * @param track The track aggregated from segments
   */
  TrackNode(ros::NodeHandle& node, Track& track, const bool turn) noexcept
    : node { node }, track { track }, turn { turn } {

    // Create markers for rviz
    createTrackMarkers();

    // Occupancy grid
//    const float griddx = 50e-3F;
//    std::array<float,track.centerSpline.dim> smin;
//    std::array<float,track.centerSpline.dim> smax;
//    std::array<uint32_t,track.centerSpline.dim> size;

//    track.getOccupancyGrid(grid.data, smin, smax, size, griddx, 20e-3F);
//    grid.header.frame_id = "map";
//    grid.header.stamp = ros::Time::now();
//    grid.info.map_load_time = grid.header.stamp;
//    grid.info.width = size[0];
//    grid.info.height = size[1];
//    grid.info.resolution = griddx;
//    grid.info.origin.position.x = smin[0];
//    grid.info.origin.position.y = smin[1];
//    grid.info.origin.position.z = 0.0;
//    grid.info.origin.orientation.x = 0.0;
//    grid.info.origin.orientation.y = 0.0;
//    grid.info.origin.orientation.z = 0.0;
//    grid.info.origin.orientation.w = 1.0;


  }

  /**
   * @brief Initializes ROS publishers, subscribers, services
   */
  inline void init() noexcept
  {
    // Setup ROS publishers and services
    trackPublisher = node.advertise<visualization_msgs::MarkerArray>("/mad/track", 4);
    trackPinPublisher = node.advertise<visualization_msgs::Marker>("/mad/trackpin", 4);
    pinSubscriber = node.subscribe("/mad/pin", 10, &TrackNode::pinCallback, this);

    graphService = node.advertiseService("/mad/get_graph", &TrackNode::graphCallback, this);
    nearestService = node.advertiseService("/mad/get_nearest", &TrackNode::nearestCallback, this);
    waypointsService = node.advertiseService("/mad/get_waypoints", &TrackNode::waypointsCallback, this);
  }

  /**
   * @brief cyclic step function
   */
  inline void step() noexcept
  {
    uint32_t newTrackSubscribersCnt = trackPublisher.getNumSubscribers();
    if (newTrackSubscribersCnt > trackSubscribersCnt) {
        // a new node has subscribed to trackPublisher
        // publish markers only once for good performance
        trackPublisher.publish(markerArray);
    }
    trackSubscribersCnt = newTrackSubscribersCnt;
  }

private:
  using ColorType = std::array<float, 4>;

  ros::NodeHandle& node;
  ros::Publisher trackPublisher;
  ros::Publisher trackPinPublisher;
  ros::Subscriber pinSubscriber;

  ros::ServiceServer graphService;
  ros::ServiceServer nearestService;
  ros::ServiceServer waypointsService;

  visualization_msgs::MarkerArray markerArray;
  const std::string ns { "map" };

  Track& track;
  const bool turn { false };

  uint32_t trackSubscribersCnt = 0U;

  visualization_msgs::Marker pinMarker;

  /**
   * @brief The service function of service /mad/get_graph
   * @param[in] request - none
   * @param[out] response - track
   * @return true on success
   */
  bool graphCallback(madmsgs::TrackGetGraphRequest&, madmsgs::TrackGetGraphResponse& response)
  {
    response.segments.clear();
    response.segments.reserve(Track::singleton->getSegments().size());
    for (std::shared_ptr<TrackSegment> segment : Track::singleton->getSegments()) {
      madmsgs::TrackSegment seg;
      if (segment->circuit == nullptr) {
        seg.circuitId = -1;
      } else {
        seg.circuitId = segment->circuit->id;
        //ROS_INFO("TrackNode.h: segment->circuit->id: %lu wird angelegt", segment->circuit->id);
        //ROS_INFO("TrackNode.h: seg.circuitId: %lu wird angelegt", segment->circuit->id);
      }
      seg.type = static_cast<uint8_t>(segment->type);
      seg.frontSegmentIds.clear();
      for (std::shared_ptr<TrackSegment> frontSegment : segment->frontPose->frontSegments) {
        seg.frontSegmentIds.push_back(frontSegment->id);
      }
      seg.rearSegmentIds.clear();
      for (std::shared_ptr<TrackSegment> rearSegment : segment->rearPose->rearSegments) {
        seg.rearSegmentIds.push_back(rearSegment->id);
      }
      response.segments.push_back(seg);
    }
    return true;
  }

  /**
   * @brief The service function of service /mad/get_nearest
   * @param[in] request - position of car
   * @param[out] response - segmentid, arclength x and dist of track
   * @return true on success
   */
  bool nearestCallback(madmsgs::TrackGetNearestRequest& request, madmsgs::TrackGetNearestResponse& response)
  {
    std::array<float, Spline::dim> s { { request.s.at(0), request.s.at(1) } };
    return Track::singleton->getNearestSegment(s, response.segment, response.x, response.dist);
  }

  /**
   * @brief The service function of service /mad/get_waypoints
   * @param[in] request alpha (0.5 for center, 0.0 for right, 1.0 for left border)
   * @param[out] response waypoints in arrays x, s1, s2
   * @return true on success
   */
  bool waypointsCallback(madmsgs::TrackGetWaypointsRequest& request, madmsgs::TrackGetWaypointsResponse& response)
  {
    bool ret = false;
    Spline spline;
    float xEnd { 0.0F };
    ret = track.sample(request.segmentSequence, request.dx, request.alpha, xEnd, spline);
    if (ret) {
      response.breaks = spline.breaks; // copy the breaks
      response.segments = spline.segmentIds; // copy the segment id of each break
      response.s1.clear();
      response.s1.reserve(spline.breaks.size());
      response.s2.clear();
      response.s2.reserve(spline.breaks.size());
      response.splineCoefs1.clear();
      response.splineCoefs1.reserve(spline.breaks.size() - 1);
      response.splineCoefs2.clear();
      response.splineCoefs2.reserve(spline.breaks.size() - 1);
      for (int i = 0; i < static_cast<int>(spline.breaks.size()); ++i) {
        response.s1.push_back(spline.vals.at(i).at(0));
        response.s2.push_back(spline.vals.at(i).at(1));
        if (i < static_cast<int>(spline.breaks.size()) - 1) {
          for (int j = Spline::order; j >= 0; --j) {
            response.splineCoefs1.push_back(spline.coefs.at(i).at(0).at(j));
            response.splineCoefs2.push_back(spline.coefs.at(i).at(1).at(j));
          }
        }
      }
      response.xEnd = xEnd;
    }
    return ret;
  }


  void createTrackMarkers()
  {
    // background
    Spline rightSpline;
    std::array<float, Spline::dim> val1 { { 0.0F, 0.0F } };
    rightSpline.pushPoint(0.0F, val1);
    std::array<float, Spline::dim> val2 { { track.s1, 0.0F } };
    rightSpline.pushPoint(track.s1, val2);
    Spline leftSpline;
    std::array<float, Spline::dim> val3 { { 0.0F, track.s2 } };
    leftSpline.pushPoint(0.0F, val3);
    std::array<float, Spline::dim> val4 { { track.s1, track.s2 } };
    rightSpline.pushPoint(track.s1, val4);
    const ColorType bgColor { { 0.0, 1.0F, 0.0F, 1.0F } };
    createSegmentAreaMarkers(rightSpline, leftSpline, bgColor, -0.03F);

    // track
    for (std::shared_ptr<TrackSegment>& segment : track.getSegments()) {
      createSegmentMarkers(segment);
    }

    // Logos
//    const float ds2 = 0.05F;
//    float s1 = 1.99F;
//    if (turn) {
//      s1 = 0.71F;
//    }
//    float s2 = 1.29F;
//    createLogoMarker("MINI-AUTO-DRIVE", s1, s2);
//    s2 -= 2.0F * ds2;
//    createLogoMarker("24 students + researchers", s1, s2);
//    s2 -= ds2;
//    createLogoMarker("Hochschule Heilbronn", s1, s2);
//    s2 -= ds2;
//    createLogoMarker("Automotive", s1, s2);
//    s2 -= ds2;
//    createLogoMarker("Systems Engineering", s1, s2);
//    s2 -= 2.0F * ds2;
//    createLogoMarker("frank.traenkle@hs-heilbronn.de", s1, s2);

//    s1 = 0.7F;
//    if (turn) {
//      s1 = 2.0F;
//    }
//    s2 = 0.85F;
//    createLogoMarker("Connect to parking system ...", s1, s2);
//    s2 -= 2.0F * ds2;
//    createLogoMarker("https://mad2.te1.hs-heilbronn.de", s1, s2);
//    s2 -= 3.0F * ds2;
//    createPinMarker("PIN 0000", s1, s2);
//    createLogoMarker("start [Alt+s]", s1, s2);
//    s2 -= ds2;
//    createLogoMarker("stop [Alt+x]", s1, s2);
//    s2 -= ds2;
//    createLogoMarker("shutdown [Alt+h]", s1, s2);
  }

  void createSegmentMarkers(std::shared_ptr<TrackSegment>& segment)
  {
    const ColorType leftColor { { 1.0F, 1.0F, 1.0F, 1.0F } };
    const ColorType rightColor { { 1.0F, 0.8F, 0.8F, 1.0F } };
    const ColorType centerColor { { 1.0F, 1.0F, 1.0F, 0.5F } };
    const ColorType surfaceColor { { 0.1F, 0.1F, 0.2F, 1.0F } };
    const ColorType parkLineColor { { 1.0F, 1.0F, 0.0F, 1.0F } };
    const ColorType parkSurfaceColor { { 0.5F, 0.2F, 0.2F, 1.0F } };
    if (segment->type == TrackSegment::RoadType::PARKING) {
      createSegmentLineMarkers(segment->rightSpline, parkLineColor, -0.02F);
      createSegmentLineMarkers(segment->centerSpline, centerColor, -0.01F);
      createSegmentLineMarkers(segment->leftSpline, parkLineColor, -0.02F);
      createSegmentAreaMarkers(segment->rightSpline, segment->leftSpline, parkSurfaceColor, -0.021F);
    } else {
      createSegmentLineMarkers(segment->rightSpline, rightColor, -0.025F);
      createSegmentLineMarkers(segment->centerSpline, centerColor, -0.01F);
      createSegmentLineMarkers(segment->leftSpline, leftColor, -0.025F);
      createSegmentAreaMarkers(segment->rightSpline, segment->leftSpline, surfaceColor, -0.011F);
    }
    createSegmentNameMarker(segment);
  }

  void createSegmentLineMarkers(const Spline& spline, const ColorType& color, const float depth)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = ns;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = markerArray.markers.size();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = ns;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = color.at(0);
    marker.color.g = color.at(1);
    marker.color.b = color.at(2);
    marker.color.a = color.at(3);
    marker.lifetime = ros::Duration();
    for (std::size_t idx = 0U; idx < spline.breaks.size(); ++idx) {
      geometry_msgs::Point p;
      p.x = spline.vals.at(idx).at(0);
      p.y = spline.vals.at(idx).at(1);
      p.z = depth;
      marker.points.push_back(p);
    }
    markerArray.markers.push_back(marker);
  }

  void createSegmentNameMarker(std::shared_ptr<TrackSegment>& segment)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = ns;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = markerArray.markers.size();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = std::to_string(segment->id);
    marker.pose.position.x = segment->rearPose->s1;
    marker.pose.position.y = segment->rearPose->s2;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.0F;
    marker.scale.y = 0.0F;
    marker.scale.z = 100e-3F;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.lifetime = ros::Duration();
    markerArray.markers.push_back(marker);
  }

  void createSegmentAreaMarkers(const Spline& rightSpline, const Spline& leftSpline, const ColorType& color, const float depth)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = ns;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = markerArray.markers.size();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = color.at(0);
    marker.color.g = color.at(1);
    marker.color.b = color.at(2);
    marker.color.a = color.at(3);
    marker.lifetime = ros::Duration();
    std::size_t rightIdx = 0U;
    std::size_t leftIdx = 0U;

    for (;;) {
      geometry_msgs::Point p;
      p.x = leftSpline.vals.at(leftIdx).at(0);
      p.y = leftSpline.vals.at(leftIdx).at(1);
      p.z = depth;
      marker.points.push_back(p);
      p.x = rightSpline.vals.at(rightIdx).at(0);
      p.y = rightSpline.vals.at(rightIdx).at(1);
      p.z = depth;
      marker.points.push_back(p);
      if (rightIdx <= leftIdx || leftIdx+1U == leftSpline.vals.size()) {
        ++rightIdx;
        if (rightIdx < rightSpline.vals.size()) {
          p.x = rightSpline.vals.at(rightIdx).at(0);
          p.y = rightSpline.vals.at(rightIdx).at(1);
          p.z = depth;
          marker.points.push_back(p);
        }
      } else if (leftIdx <= rightIdx || rightIdx+1U == rightSpline.vals.size()) {
        ++leftIdx;
        if (leftIdx < leftSpline.vals.size()) {
          p.x = leftSpline.vals.at(leftIdx).at(0);
          p.y = leftSpline.vals.at(leftIdx).at(1);
          p.z = depth;
          marker.points.push_back(p);
        }
      }
      if (rightIdx+1U == rightSpline.vals.size() && leftIdx+1U == leftSpline.vals.size()) {
        break;
      }
    }
    markerArray.markers.push_back(marker);
  }

  void createLogoMarker(const std::string& text, const float s1, const float s2)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = ns;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = markerArray.markers.size();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = text;
    marker.pose.position.x = s1;
    marker.pose.position.y = s2;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.0F;
    marker.scale.y = 0.0F;
    marker.scale.z = 50e-3F;
    marker.color.r = 0.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.lifetime = ros::Duration();
    markerArray.markers.push_back(marker);
  }

  void createPinMarker(const std::string& text, const float s1, const float s2)
  {
    visualization_msgs::Marker& marker = pinMarker;
    marker.header.frame_id = ns;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = markerArray.markers.size();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = text;
    marker.pose.position.x = s1;
    marker.pose.position.y = s2;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.0F;
    marker.scale.y = 0.0F;
    marker.scale.z = 150e-3F;
    marker.color.r = 0.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.lifetime = ros::Duration();
  }


  void pinCallback(const std_msgs::UInt32ConstPtr& msg)
  {
    char sz[16];
    std::snprintf(sz, 15, "PIN %04u", msg->data);
    pinMarker.text = std::string(sz);
    trackPinPublisher.publish(pinMarker);
  }
};

}

#endif // TRACK_NODE_H
