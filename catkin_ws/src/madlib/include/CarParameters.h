/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Car Parameters
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

#ifndef _CARPARAMETERS_H_
#define _CARPARAMETERS_H_

#include "Utils.h"
#include <array>
#include <vector>

using state_t = std::array<float, 4>; /**< The state vector type of the vehicle bicycle model */
using stateext_t = std::array<float, 7>; /**< The state vector type of the vehicle dynamics model */

/**
 * @brief The CarParameters class defines all car parameters
 */
class CarParameters
{
public:
    enum class ModelType { bicycle = 0, dynamics = 1 }; /**< Enum to switch between bicycle and dynamics model */

    /**
     * @brief CarParameters constructor used by singleton
     */
    CarParameters() noexcept;

    /**
     * @brief Use this static method to access the parameters globally
     * @return
     */
    static inline CarParameters* p()
    {
      return singleton;
    }

    // Car Model Type
    ModelType modelType { ModelType::bicycle };

    // max. number of virtual cars
    static constexpr uint32_t carCnt { 4U };
    // max. number of real cars
    static constexpr uint32_t realCarCnt { 10U };
    // size [ m ]
    const std::array<float,2> size {{ 167e-3F, 83e-3F }}; // {{ 250e-3F, 150e-3F }}; //{{ 348e-3F, 164e-3F }};
    // distance of center relative to rear axle [ m ]
    const float center { 49e-3F }; //{ 133e-3F }
    // longitudinal dynamics gain [ m/s ]
    const float k { 2.51F };
    // open-loop longitudinal dynamics time constant [ s ]
    const float T { 316e-3F };
    // open-loop dead time of longitudinal dynamics [ s ]
    const float uTt { 60e-3F };
    // open-loop dead time of steering [ s ]
    const float deltaTt { uTt };
    // image processing dead time
    const float outputTt { 40e-3F };
    // total dead time [ s ]
    const float Tt { uTt + outputTt };
    // longitudinal friction
    const float uFrictionPos { 0.2F };
    const float uFrictionNeg { -uFrictionPos };
    // delta max [ rad ]
    const float deltaMax { modbas::Utils::deg2rad(21.58F) };
    // wheel base [ m ]
    const float l { 99e-3F }; //{ 0.260F };
    // minimum speed [ m/s ]
    const float speedMin { 0.15F };
    // maximum speed [ m/s ]
    float speedMax { k };
    // maximum speed limit [ m/s ]
    const float speedMaxLimit { speedMax };
    // steering adaption clockwise
    const float adaptcw { 0.000F };
    // steering adaption counterclockwise
    const float adaptccw { 0.049F };

    // planning
    const int32_t planHorizon = 120; // number of waypoints
    const bool replanEnable = false; // enable replanning during control

    // dynamics model
    const float m { 132e-3F }; // mass [ kg ]
    const float J { 192e-6F }; // moment of inertia [ kg*m^2 ]
    const float lf { 49e-3F }; // distance COG to front axle [ m ]
    const float lr { 50e-3F }; // distance COG to rear axle [ m ]

    // tyre coefficients of Pacejka Model (Magic Formula
    const float Br { 0.0014F }; // rear: stiffness factor
    const float Cr { 16.398F }; // rear: shape factor
    const float Dr { 135.9653F }; // rear: peak value
    const float Bf { 0.025F }; // front: stiffness factor
    const float Cf { 114.3442F }; // front: shape factor
    const float Df { 0.8136F }; // front: peak value

    // lateral control
    const float lateralAccMax { 3.0F }; // maximum lateral acceleration [ m/s^2 ]
    const float lateralTw { 200e-3F}; // closed-loop longitudinal dynamics time constant [ s ]

    // speed control
    const float speedLookahead { T * 0.4F }; // lookahead for speed control [ s ]
    const float speedKp { 0.238F }; // speed control gain
    const float speedTi { 140e-3F }; // speed control integral time

    // sampling time [ s ]
    const float Ta { 2e-3F };

    // downsample for car marker publishing
    const int32_t realtimeDownsample = static_cast<int32_t>(50e-3F / Ta);
    const int32_t nonrealtimeDownsample = static_cast<int32_t>(50e-3F / Ta);

    // downsample for carouputs message of madvision
    const int32_t caroutputsDownsample = 10; // so that caroutputsext rate is 50Hz

    // downsample for carstate message of madrc2
    const int32_t carstateDownsample = 500; // so that caroutputsext rate is 1Hz

    // vision sampling time [ s ]
    const float Tva = static_cast<float>(caroutputsDownsample) * Ta;

    // user mission limits
    const float vmax { 2.0F };
    const float vmin { 0.1F };
    const uint32_t lapCountMax { 20U };

    // message names
    const std::vector<std::string> carState {
      { "connected", "disconnect", "error", "drive",
        "drive_slow", "charge" }
    };

    const std::vector<std::string> carErr {
      { "none", "calib", "overcurrent", "rn4020_detect_firmware",
        "steer_control", "carin_msg_watchdoc", "unknown" }
    };

    const std::vector<std::string> maneuverState {
      { "waiting", "safetyhalt", "safetystop", "driving",
        "halt", "charging", "locating", "batteryempty?", "error" }
    };

    // initial conditions v0, s10, s20, psi0
    state_t x0 { { 0.0F, 2.0F, 1.0F, 0.0F } };


    // car id
    int32_t carid = 0;

private:
    static CarParameters* singleton; /**< This class is a singleton */
};

#endif
