/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * BUGA Track Map
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

#include "ros/ros.h"
#include "TrackNode.h"

using namespace std;


/**
 * @brief createCornerCurve creates corner
 * @param track
 * @param pose
 * @param width
 * @param r
 */
static void createCornerCurve(modbas::Track& track, std::shared_ptr<modbas::TrackPose>& pose,
                              const float width, const float a, const float turn)
{
  // circle
  //  const float r { 0.26F }; // radius of driving circles [ m ]
  //  track.addSegment(std::make_shared<modbas::CircleSegment>(
  //          new modbas::CircleSegment(pose, width, r, 0.5F * modbas::Utils::pi));

  // clothoids
  track.addSegment(std::make_shared<modbas::ClothoidSegment>(pose, width, a, turn * 0.25F * modbas::Utils::pi,
                                                 modbas::ClothoidSegment::ClothoidType::CLOSING));
  track.addSegment(std::make_shared<modbas::ClothoidSegment>(
                     pose, width, a, turn * 0.25F * modbas::Utils::pi,
                                                 modbas::ClothoidSegment::ClothoidType::OPENING));
}

/**
 * @brief computeCornerCurveSize computes size of corner
 * @param dx
 * @param width
 * @param a
 * @param turn
 * @param w1
 * @param w2
 */
static void computeCornerCurveSize(const float dx, const float width, const float a, const float turn, float& w1, float& w2)
{
  modbas::Track track { 3.0F, 3.0F, dx };
  std::shared_ptr<modbas::TrackPose> pose {
    std::make_shared<modbas::TrackPose>(0.0F, 0.0F, 0.0F, 0.0F) };
  track.addPose(pose);
  createCornerCurve(track, pose, width, a, turn);
  w1 = pose->s1;
  w2 = pose->s2;
}

/**
 * @brief the main function
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char* argv[])
{

//    ROS_INFO("Press any key to continue ... TrackParkingNode");
//    getchar();

  // Set up ROS
  ros::init(argc, argv, "track_test_node");
  ros::NodeHandle nh { "~" };
  bool turn { false };
  nh.param<bool>("turn", turn, false);


  // Mini Auto Drive track
  const float turnFloat { turn ? -1.0F : 1.0F };
  const float dx { 0.02F }; // sampling step size for graphics [ m ]
  const float a1total { 2.700F }; // total surface width [ m ]
  const float a2total { 1.800F }; // total surface height [ m ]
  const float a1boundary { 0.05F }; // boundary for markers [ m ]
  const float a2boundary { 0.05F }; // boundary for markers [ m ]
  const float a1 { a1total - 2.0F * a1boundary };
  const float a2 { a2total - 2.0F * a2boundary };
  const float width { 0.20F }; // track width [ m ]
  const float clothoidA { 8.0F }; // clothoid parameter
  const uint32_t parkCnt { 6U }; // number of parking spots
  const float parkWidth { 0.15F }; // radius of parking entry circles [ m ]
  const float parkRadius { 0.4F }; // radius of parking entry circles [ m ]
  const float parkAngle { modbas::Utils::deg2rad(75.0F) }; // parking spot angle [ rad ]
  const float parkLength { 0.38F }; // length of parking space [ m ]

  // compute corner curve size
  float w1 { 0.0F };
  float w2 { 0.0F };
  computeCornerCurveSize(dx, width, clothoidA, turnFloat, w1, w2);

  // parking space parameters
  const float parkDisplacement { 0.0F }; // correction for vision displacement [ m ]
  const float parkDistance { (a1 - 2.0F * w1 - width) / (static_cast<float>(parkCnt) * 0.5F) }; // distance between parking spots [ m ]
  std::array<std::shared_ptr<modbas::TrackPose>, parkCnt> parkPose; // starting poses of parking spaces

  modbas::Track track { a1total, a2total, dx };
  track.initCircuit();
  std::shared_ptr<modbas::TrackPose> pose {
    std::make_shared<modbas::TrackPose>(0.0F, turn ? (a1total - (a1boundary + 0.5F * width)) : (a1boundary + 0.5F * width),
                         a2boundary + 0.5F * a2, -0.5F * modbas::Utils::pi) };
  track.addPose(pose);
  // Straight 0 sequment
  track.addSegment(std::make_shared<modbas::StraightSegment>(
          pose, width, 0.5F * a2 - w1 - 0.5F * width));
  // start corner sequment 1 and 2
  createCornerCurve(track, pose, width, clothoidA, turnFloat);
  if (parkDisplacement > 0.0F) {
    track.addSegment(std::make_shared<modbas::StraightSegment>(
            pose, width, parkDisplacement));
  }
  for (uint32_t i = 0U; i < parkCnt / 2U; ++i) {
    if (turn) {
      parkPose.at(parkCnt / 2U - 1U - i) = pose;
    } else {
      parkPose.at(i) = pose;
    }
    float dist = parkDistance;
    if (i == parkCnt / 2U - 1) {
      dist -= parkDisplacement;
    }
    track.addSegment(std::make_shared<modbas::StraightSegment>(
            pose, width, dist));
  }
  createCornerCurve(track, pose, width, clothoidA, turnFloat);
  track.addSegment(std::make_shared<modbas::StraightSegment>(
          pose, width, a2 - 2.0F * w1 - width));
  createCornerCurve(track, pose, width, clothoidA, turnFloat);
  if (parkDisplacement > 0.0F) {
    track.addSegment(std::make_shared<modbas::StraightSegment>(
            pose, width, parkDisplacement));
  }
  for (uint32_t i = parkCnt / 2U; i < parkCnt; ++i) {
    if (turn) {
      parkPose.at(parkCnt / 2U * 3U - 1U  - i) = pose;
    } else {
      parkPose.at(i) = pose;
    }
    float dist = parkDistance;
    if (i == parkCnt - 1) {
      dist -= parkDisplacement;
    }
    track.addSegment(std::make_shared<modbas::StraightSegment>(
                  pose, width, dist));
  }
  createCornerCurve(track, pose, width, clothoidA, turnFloat);
  track.addSegment(std::make_shared<modbas::StraightSegment>(
          pose, width, 0.5F * a2 - w1 - 0.5F * width));

  if (!track.finalizeCircuit()) {
    ROS_ERROR("finalizing lap failed");
  }

  // parking spaces
  for (uint32_t i = 0U; i < parkCnt; ++i) {
    track.addSegment(std::make_shared<modbas::CircleSegment>(
            parkPose.at(i), parkWidth, parkRadius, turnFloat * parkAngle, modbas::TrackSegment::RoadType::ONELANE));
    track.addSegment(std::make_shared<modbas::StraightSegment>(
            parkPose.at(i), parkWidth, parkLength, modbas::TrackSegment::RoadType::PARKING));
  }

  // Start ROS node
  modbas::TrackNode node(nh, track, turn);
  node.init();
  ros::Rate rate(1.0F);

  // ROS loop
  while (ros::ok()) {
      node.step();
      rate.sleep();
      ros::spinOnce();
  }
}



