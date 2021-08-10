/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Car Display for SiL-Testing or Augmented Reality
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

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "madmsgs/CtrlReference.h"
#include "madmsgs/DriveManeuver.h"
#include "madmsgs/DriveManeuverState.h"
#include "madmsgs/MissionState.h"
#include "CarParameters.h"

namespace modbas {

/**
 * @brief The CarDisplayNode class
 */
class CarDisplayNode
{
public:
  /**
   * @brief Constructor
   */
  CarDisplayNode() noexcept
  {
    readParameters();
    maneuverStateMsg.state = maneuverStateMsg.STATE_LOCATING;
    outputsSub = node.subscribe("/mad/caroutputs", 10, &CarDisplayNode::outputsCallback, this);
    outputsextSub = node.subscribe("/mad/caroutputsext", 10, &CarDisplayNode::outputsextCallback, this);
    ctrlreferenceSub = node.subscribe("/mad/ctrlreference", 10, &CarDisplayNode::ctrlreferenceCallback, this);
    for (uint32_t idx = 0U; idx < CarParameters::carCnt; ++idx) {
      maneuverSubs.push_back(node.subscribe("/mad/car" + std::to_string(idx) + "/navi/maneuver", 10, &CarDisplayNode::maneuverCallback, this));
      maneuverStateSubs.push_back(node.subscribe("/mad/car" + std::to_string(idx) + "/navi/maneuverstate", 10, &CarDisplayNode::maneuverStateCallback, this));
    }
    missionStateSub = node.subscribe("/mad/missionstate", 10, &CarDisplayNode::missionStateCallback, this);
    markerPub = node.advertise< visualization_msgs::MarkerArray >("markers", 10);
    createMarkers();
  }

  /**
   * @brief Cyclic step function
   */
  inline void step() noexcept
  {
    ros::Time time { ros::Time::now() };

    // TF broadcaster
    static visualization_msgs::Marker::_ns_type ns = carShortName();
    static tf::TransformBroadcaster broadcaster;
    tf::Transform statesTf;
//    if (maneuverStateMsg.state == maneuverStateMsg.STATE_LOCATING) {
//      // hide car
//      outputsMsg.s.at(0) = -1.0F;
//      outputsMsg.s.at(1) = -1.0F;
//    }
    statesTf.setOrigin({ outputsMsg.s.at(0), outputsMsg.s.at(1), 0.0});
    auto quat { tf::createQuaternionMsgFromYaw(outputsMsg.psi) };
    statesTf.setRotation({ quat.x, quat.y, quat.z, quat.w });
    broadcaster.sendTransform(tf::StampedTransform(statesTf, time, "map",
                                                   ns));

    // car marker
    markerArray.markers.at(0).scale.x = outputsextMsg.v * 0.5F;
    markerArray.markers.at(2).text = carName();

    // car reference marker
    markerArray.markers.at(1).scale.x = ctrlreferenceMsg.v * 0.5F;
    markerArray.markers.at(1).pose.position.x = ctrlreferenceMsg.s.at(0);
    markerArray.markers.at(1).pose.position.y = ctrlreferenceMsg.s.at(1);
    markerArray.markers.at(1).pose.position.z = 0.0;
    markerArray.markers.at(1).pose.orientation = tf::createQuaternionMsgFromYaw(ctrlreferenceMsg.psi);

    // target segment marker
    markerArray.markers.at(5).pose.position.x = missionStateMsg.targetSegmentS.at(0);
    markerArray.markers.at(5).pose.position.y = missionStateMsg.targetSegmentS.at(1);

    // publish markers
    for (auto& marker : markerArray.markers) {
        marker.header.stamp = time;
    }
    markerPub.publish(markerArray);
  }

private:
  ros::NodeHandle node { "~" }; /**< init ROS node, relative namespace from launch file */
  std::vector<float> colorRGB { {1.0F, 0.3F, 0.3F} }; /** marker color */
  ros::Subscriber outputsSub;
  ros::Subscriber outputsextSub;
  ros::Subscriber ctrlreferenceSub;
  std::vector<ros::Subscriber> maneuverSubs;
  std::vector<ros::Subscriber> maneuverStateSubs;
  ros::Subscriber missionStateSub;
  ros::Publisher markerPub;
  visualization_msgs::MarkerArray markerArray;
  madmsgs::CarOutputs outputsMsg;
  madmsgs::CarOutputsExt outputsextMsg;
  madmsgs::CtrlReference ctrlreferenceMsg;
  madmsgs::DriveManeuver maneuverMsg;
  madmsgs::DriveManeuverState maneuverStateMsg;
  madmsgs::MissionState missionStateMsg;

  /**
   * @brief readParameters reads params from launch file
   */
  void readParameters()
  {
    node.param<int>("carid", CarParameters::p()->carid, 0);
    if (!(node.hasParam("colorRGB") && node.getParam("colorRGB", colorRGB) && colorRGB.size() == 3)) {
        ROS_INFO("%s: matching parameter colorRGB specified", node.getNamespace().c_str());
    }
  }

  /**
   * @brief createCarNameMarker creates name marker
   * @param marker
   * @param ns
   * @param id
   */
  void createCarNameMarker(visualization_msgs::Marker& marker,
                           visualization_msgs::Marker::_ns_type ns,
                           visualization_msgs::Marker::_id_type id,
                           const std::vector<float>& colorRGB)
  {
      marker.header.frame_id = ns;
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = carName();
      marker.pose.position.x = 0.2;
      marker.pose.position.y = 0.1;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      marker.scale.x = 0.0F;
      marker.scale.y = 0.0F;
      marker.scale.z = 100e-3F;
      marker.color.r = colorRGB[0];
      marker.color.g = colorRGB[1];
      marker.color.b = colorRGB[2];
      marker.color.a = 1.0F;
      marker.lifetime = ros::Duration();
  }

  /**
   * @brief createCarBoxMarker creates box marker
   * @param marker
   * @param ns
   * @param id
   * @param colorRGB
   */
  void createCarBoxMarker(visualization_msgs::Marker& marker,
                                 visualization_msgs::Marker::_ns_type ns,
                                 visualization_msgs::Marker::_id_type id,
                                 const std::vector<float>& colorRGB)
  {
//    marker.header.frame_id = ns;
//    marker.header.stamp = ros::Time::now();
//    marker.ns = ns;
//    marker.id = id;
//    marker.type = visualization_msgs::Marker::CUBE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.text = ns;
//    marker.pose.position.x = 0.0;
//    marker.pose.position.y = 0.0;
//    marker.pose.position.z = 0.0;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 0.0;
//    marker.scale.x = CarParameters::p()->size[0];
//    marker.scale.y = CarParameters::p()->size[1];
//    marker.scale.z = 50e-3F;
//    marker.color.r = colorRGB[0];
//    marker.color.g = colorRGB[1];
//    marker.color.b = colorRGB[2];
//    marker.color.a = 0.5F;
//    marker.lifetime = ros::Duration();
    marker.header.frame_id = ns;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://madcar/data/R8_skeletal.dae";
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = ns;
    marker.pose.position.x = CarParameters::p()->center; // rear axle position
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.707;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.707;
    const double scale = 0.0004;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = colorRGB[0];
    marker.color.g = colorRGB[1];
    marker.color.b = colorRGB[2];
    marker.color.a = 1.0F;
    marker.lifetime = ros::Duration();
  }

  /**
   * @brief createCarArrowMarker creates arrow marker for velocity
   * @param marker
   * @param ns
   * @param id
   * @param colorRGB
   */
   void createCarArrowMarker(visualization_msgs::Marker& marker,
                                   visualization_msgs::Marker::_ns_type ns,
                                   visualization_msgs::Marker::_id_type id,
                                   const std::vector<float>& colorRGB)
  {
      marker.header.frame_id = ns;
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = ns;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.05;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      marker.scale.x = 1e-3F;
      marker.scale.y = 30e-3F;
      marker.scale.z = 30e-3F;
      marker.color.r = colorRGB[0];
      marker.color.g = colorRGB[1];
      marker.color.b = colorRGB[2];
      marker.color.a = 1.0F;
      marker.lifetime = ros::Duration();
  }


  void createManeuverMarker(visualization_msgs::Marker& marker,
                            visualization_msgs::Marker::_ns_type ns,
                            visualization_msgs::Marker::_id_type id,
                            const std::vector<float>& colorRGB)
  {
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0.0F;
      marker.pose.position.y = 0.0F;
      marker.pose.position.z = 0.0F;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.02;
      marker.color.r = colorRGB[0];
      marker.color.g = colorRGB[1];
      marker.color.b = colorRGB[2];
      marker.color.a = 0.9F;
      marker.lifetime = ros::Duration();
  }


  void createTargetSegmentMarker(visualization_msgs::Marker& marker,
                            visualization_msgs::Marker::_ns_type ns,
                            visualization_msgs::Marker::_id_type id,
                            const std::vector<float>& colorRGB)
  {
      missionStateMsg.targetSegmentS.at(0) = -1.0F;
      missionStateMsg.targetSegmentS.at(1) = -1.0F;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = missionStateMsg.targetSegmentS.at(0);
      marker.pose.position.y = missionStateMsg.targetSegmentS.at(1);
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.02;
      marker.color.r = colorRGB[0];
      marker.color.g = colorRGB[1];
      marker.color.b = colorRGB[2];
      marker.color.a = 1.0F;
      marker.lifetime = ros::Duration();
  }

  void updateManeuverMarker(visualization_msgs::Marker& marker)
  {
    marker.points.clear();
    for (int i = 0; i < static_cast<int>(maneuverMsg.s1.size()); ++i) {
        geometry_msgs::Point p;
        p.x = maneuverMsg.s1.at(i);
        p.y = maneuverMsg.s2.at(i);
        p.z = 0.01;
        marker.points.push_back(p);
    }
  }

  /**
   * @brief createMarkers creates marker array
   */
  void createMarkers()
  {
    const std::string ns = carShortName();
    visualization_msgs::Marker markerArrow;
    createCarArrowMarker(markerArrow, ns, 0, colorRGB);
    markerArrow.color.r = 1.0F;
    markerArrow.color.g = 1.0F;
    markerArrow.color.b = 1.0F;
    markerArray.markers.push_back(markerArrow);

    visualization_msgs::Marker markerRefArrow;
    createCarArrowMarker(markerRefArrow, "map", 1, colorRGB);
    markerArray.markers.push_back(markerRefArrow);

    visualization_msgs::Marker markerName;
    createCarNameMarker(markerName, ns, 2, colorRGB);
    markerArray.markers.push_back(markerName);

    visualization_msgs::Marker markerBox;
    createCarBoxMarker(markerBox, ns, 3, colorRGB);
    markerArray.markers.push_back(markerBox);

    visualization_msgs::Marker markerManeuver;
    createManeuverMarker(markerManeuver, ns, 4, colorRGB);
    markerArray.markers.push_back(markerManeuver);

    visualization_msgs::Marker markerTargetSegment;
    createTargetSegmentMarker(markerTargetSegment, ns, 5, colorRGB);
    markerArray.markers.push_back(markerTargetSegment);

    markerPub.publish(markerArray);
    for (auto& marker : markerArray.markers) {
        marker.action = visualization_msgs::Marker::MODIFY;
    }
  }

  /**
   * @brief outputsCallback triggers display
   * @param msg CarOutputs message
   */
  void outputsCallback(const madmsgs::CarOutputsConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      outputsMsg = *msg;
    }
  }

  void outputsextCallback(const madmsgs::CarOutputsExtConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      outputsextMsg = *msg;
    }
  }

  void ctrlreferenceCallback(const madmsgs::CtrlReferenceConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      ctrlreferenceMsg = *msg;
    }
  }

  void maneuverCallback(const madmsgs::DriveManeuverConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      maneuverMsg = *msg;
      updateManeuverMarker(markerArray.markers.at(4));
    }
  }

  void maneuverStateCallback(const madmsgs::DriveManeuverStateConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      maneuverStateMsg = *msg;
    }
  }

  void missionStateCallback(const madmsgs::MissionStateConstPtr& msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      missionStateMsg = *msg;
    }
  }


  std::string floatToString(const float x)
  {
    float y = 1000.0F * x;
    if (std::fabs(y) < 10.0F) {
      char buf[4];
      std::snprintf(buf, sizeof(buf), "%+2.0f", y);
      return std::string(buf);
    } else {
      return "";
    }
  }

  std::string carShortName()
  {
    return "car" + std::to_string(CarParameters::p()->carid);
  }

  std::string carName()
  {
      std::string state { CarParameters::p()->maneuverState.at(maneuverStateMsg.state) };
      if (maneuverStateMsg.batteryLow) {
        if (maneuverStateMsg.state == maneuverStateMsg.STATE_CHARGING) {
          state = "batterylow";
        } else {
          state += "low";
        }
      }
      std::string label { std::to_string(CarParameters::p()->carid) + ":"
              + state
              + floatToString(maneuverStateMsg.ex)
              + "," + floatToString(maneuverStateMsg.ey) };
      return label;
  }

};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "madcardisplay");
  modbas::CarDisplayNode node;
  ros::Rate rate { 15.0F };
  while (ros::ok()) {
    node.step();
    rate.sleep();
    ros::spinOnce();
  }
  return EXIT_SUCCESS;
}
