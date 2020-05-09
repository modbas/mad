/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Random Number Generator
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

#ifndef _RANDOM_H_
#define _RANDOM_H_

#include "Utils.h"
#include <array>
#include <random>

namespace modbas {

/**
 * @brief The Random Generator class
 */
class Random
{
public:
  /**
     * @brief Random constructor used by singleton
     */
  Random() noexcept;

  static std::mt19937& generator()
  {
    return singleton->gen;
  }

private:
  static Random* singleton; /**< This class is a singleton */
  std::random_device rd;
  std::mt19937 gen { rd() };
};

}

#endif
