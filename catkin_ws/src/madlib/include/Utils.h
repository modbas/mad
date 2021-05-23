/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Common utils for path following
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

#ifndef _MODBAS_UTILS_H
#define _MODBAS_UTILS_H

#include <cmath>

namespace modbas {

/**
 * @brief The Utils struct
 */
struct Utils {
  constexpr static float pi { M_PIf32 }; // pi in float precision
  constexpr static double piDbl { M_PI }; // pi in double precision

  /**
   * @brief deg2rad converts degrees to rad
   * @param[in] deg Degrees
   * @return rad
   */
  static constexpr float deg2rad(const float deg)
  {
    return deg * pi / 180.0F;
  }

  /**
   * @brief rad2deg converts rad to degrees
   * @param[in] rad Rad
   * @return degrees
   */
  static constexpr float rad2deg(const float rad)
  {
    return rad * 180.0F / pi;
  }

  /**
   * @brief normalizeRad normales rad to circle
   * @param[in] rad Rad
   * @return rad in the range of [-pi; pi]
   */
  static constexpr double normalizeRad(const double rad)
  {
    return rad - 2.0 * piDbl * std::floor(rad / (2.0 * pi) + 0.5);
  }

  /**
   * @brief normalizeRad normales rad to circle
   * @param[in] rad Rad
   * @return rad in the range of [-pi; pi]
   */
  static constexpr float normalizeRad(const float rad)
  {
    return rad - 2.0F * pi * std::floor(rad / (2.0F * pi) + 0.5F);
  }

  /**
   * @brief sqr computes square (^2)
   * @param[in] x Number
   * @return x^2
   */
  static constexpr float sqr(const float x)
  {
    return x * x;
  }

  /**
   * @brief sqr computes square (^2)
   * @param[in] x Number
   * @return x^2
   */
  static constexpr double sqr(const double x)
  {
    return x * x;
  }

  /**
   * @brief compareRads compares angles in rad with tolerance
   * @param[in] rad1 first angle
   * @param[in] rad2 second angle
   * @param[in] abstol absolute tolerance
   * @return true if rad1 and rad2 are equal with tolerance
   */
  static constexpr bool compareRads(const float rad1, const float rad2, const float abstol)
  {
    const float diff = normalizeRad(rad1 - rad2);
    return (diff >= -abstol) && (diff <= abstol);
  }

};

}


#endif // _MODBAS_UTILS_H
