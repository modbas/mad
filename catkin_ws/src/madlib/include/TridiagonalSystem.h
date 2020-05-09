/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Linear equation system in tri-diagonal form
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

#ifndef _TRIDIAGONALSYSTEM_H
#define _TRIDIAGONALSYSTEM_H

#include <vector>
#include <array>

namespace modbas {

template <int dim> class TridiagonalSystem
{
public:
    TridiagonalSystem(const int n)
        : A(n), b(n), x(n)
    {
    }

    std::vector<std::array<float,dim>>& solve()
    {
        int n = b.size();
        A[0][2] /= A[0][1];
        for (int j = 0; j < dim; ++j) {
            b[0][j] /= A[0][1];
        }
        for (int i = 1; i < n; ++i) {
            float div = A[i][1] - A[i][0] * A[i-1][2];
            A[i][2] /= div;
            for (int j = 0; j < dim; ++j) {
                b[i][j] = (b[i][j] - A[i][0] * b[i-1][j]) / div;
            }
        }
        for (int j = 0; j < dim; ++j) {
            x[n-1][j] = b[n-1][j];
        }
        for (int i = n-2; i >= 0; --i) {
            for (int j = 0; j < dim; ++j) {
                x[i][j] = b[i][j] - A[i][2] * x[i+1][j];
            }
        }

        return x;
    }

    std::vector<std::array<float, 3>> A; // 3 diagonals
    std::vector<std::array<float, dim>> b; // dim splines
    std::vector<std::array<float, dim>> x; // dim splines
};

}
#endif // _TRIDIAGONALSYSTEM_H
