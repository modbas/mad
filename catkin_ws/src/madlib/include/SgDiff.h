/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Savitzky-Golay Differentiator
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

#ifndef _SGDIFF_H
#define _SGDIFF_H

#include <deque>
#include <array>

namespace modbas {

class SgDiff
{
  public:
    SgDiff() noexcept
    {
    }

    SgDiff(const size_t order) noexcept
      : order { order }
    {
    }

    void reset() noexcept
    {
      x.clear();
    }

    float filter(const float u) noexcept
    {
      x.push_back(u);
      while (x.size() > order) {
        x.pop_front();
      }
      const size_t order = x.size();
      if (order >= 2U) {
        // use backward differences or Savitzky-Golay
        y = 0.0F;
        for (uint8_t idx = 0U; idx < order; ++idx) {
          y += b[order-1][idx] * x.at(idx);
        }
      } // else: return previous value

      return y;
    }

  private:
    //const std::array<float, 4> b { { 0.45F, -0.85F, -0.65F, 1.05F } };
    static constexpr size_t orderMax = 7U;
    const float b[orderMax][orderMax] =
        { { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },  // 1 point --> no differentation possible
          { -1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }, // 2 points --> backward differences
          { -0.5F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F }, // 3 points --> sgsdf(-2:0, 1, 1, 0, 0)
          { 0.45F, -0.85F, -0.65F, 1.05F, 0.0F, 0.0F, 0.0F }, // 4 points --> sgsdf(-3:0, 2, 1, 0, 0)
          { 0.371428571428572F, -0.385714285714287F, -0.571428571428573F, -0.185714285714286F, 0.771428571428572F, 0.0F, 0.0F }, // 5 points --> sgsdf(-4:0, 2, 1, 0, 0)
          { 0.303571428571430F, -0.175000000000000F, -0.385714285714287F, -0.328571428571429F, -0.003571428571429F, 0.589285714285715F, 0.0F }, // 6 points -> sgsdf(-5:0, 2, 1, 0, 0)
          { 0.25F, -0.071428571428571F, -0.25F, -0.285714285714285F, -0.178571428571428F, 0.071428571428571F, 0.464285714285713F } }; // 7 points -> sgsdf(-6:0, 2, 1, 0, 0)
    const size_t order { orderMax };
    std::deque<float> x;
    float y = 0.0F;
};

}

#endif // _SGDIFF_H
