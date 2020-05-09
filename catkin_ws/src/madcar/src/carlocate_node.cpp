/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Car Localization
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
  * @file carlocate_node.cpp
  */

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdint>
#include <cmath>
#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "CarParameters.h"
#include "SgDiff.h"
#include "Vector.h"
#include "Utils.h"
#include "carlocate-tp.h"

namespace modbas {

/**
 * @brief The CarLocateNode class to estimate car velocity
 */
class CarLocateNode
{
public:
  /**
   * @brief Constructor
   */
  CarLocateNode() noexcept
  {
    readParameters();
    outputsSub = node.subscribe("/mad/caroutputs",
                                2 * CarParameters::p()->carCnt,
                                &CarLocateNode::outputsCallback, this,
                                ros::TransportHints().udp());
    outputsExtPub = node.advertise<madmsgs::CarOutputsExt>("/mad/caroutputsext",
                                                           2 * CarParameters::p()->carCnt);
    posePub = node.advertise<geometry_msgs::Pose>("pose", 1);
    diagUpdater.setHardwareID(node.getNamespace());
    diagUpdater.add("locate", this, &CarLocateNode::diagDiff);
  }

private:
  ros::NodeHandle node { "~" }; /**< init ROS node, relative namespace from launch file */
  ros::Subscriber outputsSub;
  ros::Publisher outputsExtPub;
  ros::Publisher posePub;
  madmsgs::CarOutputsExt outputsExtMsg;
  diagnostic_updater::Updater diagUpdater;
  std::array<SgDiff, 2> sgdiff;
  uint64_t lastFrameId = 0ULL;
  double lastTime = 0.0;
  bool diffSuccess = true;
  bool vSuccess = true;
  float lastV = 0.0F; // speed from last frame to validate acceleration
  const float vValidMax = 3.0F; // maximum valid speed [ m/s ]
  const float aValidMax = 10.0F; // maximum valid acceleration [ m/s^2 ]
  const uint32_t poseDownsampleMax = static_cast<uint32_t>(200e-3F / CarParameters::p()->Tva + 0.5F);
  uint32_t poseDownsampleCtr = 1U;

  /**
   * @brief readParameters reads params from launch file
   */
  void readParameters()
  {
    node.param<int>("carid", CarParameters::p()->carid, 0);
  }

  /**
   * @brief inputsCallback triggers simulation step
   * @param msg CarInputs message
   */
  void outputsCallback(const madmsgs::CarOutputsConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      tracepoint(carlocate, cp, msg->cameraFrameId, msg->carid, 2);
      ros::Time stamp = msg->header.stamp;
      ros::Duration delay = ros::Time::now() - stamp;
      if (msg->cameraFrameId != 0U && (msg->cameraFrameId != lastFrameId+1U)) {
        // message loss
        for (uint8_t idx = 0U; idx < sgdiff.size(); ++idx) {
          sgdiff[idx].reset();
        }
        diffSuccess = false;
        ROS_INFO("LOCATE car %u: /mad/caroutputs message lost", CarParameters::p()->carid);
      } else {
        diffSuccess = true;
      }
      Vector<float> vv { 0.0F, 0.0F };
      for (uint8_t idx = 0U; idx < sgdiff.size(); ++idx) {
        vv.s[idx] = sgdiff[idx].filter(msg->s[idx]) / CarParameters::p()->Tva;
      }
      lastFrameId = msg->cameraFrameId;
      lastTime = msg->header.stamp.toSec();

      tracepoint(carlocate, cp, msg->cameraFrameId, msg->carid, 3);
      outputsExtMsg.header.stamp = ros::Time::now();
      outputsExtMsg.cameraFrameId = msg->cameraFrameId;
      outputsExtMsg.carid = CarParameters::p()->carid;
      outputsExtMsg.v = vv.abs();
      outputsExtMsg.s = msg->s;
      outputsExtMsg.psi = msg->psi;
      outputsExtMsg.beta = Utils::normalizeRad(std::atan2(vv.s[1], vv.s[0]) - msg->psi);
      if (std::fabs(outputsExtMsg.beta) > Utils::pi * 0.5F) {
        outputsExtMsg.v = -outputsExtMsg.v;
      }
      if (std::fabs(outputsExtMsg.v) > vValidMax) {
        // invalid v due to image proc faults due to high brightness
        vSuccess = false;
        ROS_ERROR("LOCATE car %u: invalid v due to image proc faults", CarParameters::p()->carid);
      } else if (diffSuccess &&
                 std::fabs(outputsExtMsg.v - lastV) > aValidMax * CarParameters::p()->Tva) {
        // invalid acceleration due to image proc faults due to high brightness
        vSuccess = false;
        ROS_ERROR("LOCATE car %u: invalid acceleration due to image proc faults", CarParameters::p()->carid);
      } else {
        outputsExtPub.publish(outputsExtMsg);
        vSuccess = true;
      }
      lastV = outputsExtMsg.v;
      diagUpdater.update();

      // outputs pose message for Unreal
      --poseDownsampleCtr;
      if (poseDownsampleCtr == 0U) {
        poseDownsampleCtr = poseDownsampleMax;
        geometry_msgs::Pose poseMsg;
        poseMsg.position.x = msg->s.at(0);
        poseMsg.position.y = msg->s.at(1);
        poseMsg.position.z = 0.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, msg->psi);
        poseMsg.orientation.x = quaternion.x();
        poseMsg.orientation.y = quaternion.y();
        poseMsg.orientation.z = quaternion.z();
        poseMsg.orientation.w = quaternion.w();
        posePub.publish(poseMsg);
      }
    }
  }

  void diagDiff(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (!diffSuccess) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "/mad/caroutputs message lost");
    } else if (!vSuccess) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "speed invalid due to image processing fault");
    } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "no message loss");
    }
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "madcarlocate");
  modbas::CarLocateNode node;
  ros::spin();

  return EXIT_SUCCESS;
}

