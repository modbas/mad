/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Cubic Spline
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

#include "Spline.h"
#include "TridiagonalSystem.h"
#include <cmath>
#include <cfloat>
#include <iostream>
#include <ros/ros.h>

namespace modbas {

Spline::Spline() noexcept
{
}

Spline::Spline(const std::vector<float>& breaks,
               const std::vector<float>& vals0,
               const std::vector<float>& vals1,
               const std::vector<uint32_t>& segmentIds) noexcept
  : breaks(breaks), vals(vals0.size()), segmentIds(segmentIds)
{
  for (int i = 0; i < static_cast<int>(vals0.size()); ++i) {
    vals[i][0] = vals0[i];
    vals[i][1] = vals1[i];
  }
  computeCoefficients();
}

void Spline::reserve(const int pieces_cnt)
{
  breaks.reserve(pieces_cnt + 1);
  vals.reserve(pieces_cnt + 1);
  segmentIds.reserve(pieces_cnt + 1);
}

void Spline::pushPoint(const float brk, const std::array<float, dim>& val, const uint32_t segmentId)
{
  breaks.push_back(brk);
  vals.push_back(val);
  segmentIds.push_back(segmentId);
}

void Spline::computeCoefficients(const BoundaryCondition bc)
{
  boundaryCondition = bc;
  if (boundaryCondition == BoundaryCondition::periodic) {
    computeCoefficientsPeriodic();
  } else {
    computeCoefficientsNatural();
  }
}


void Spline::computeCoefficientsNatural()
{
  int n = breaks.size(); // linear equation system size

  TridiagonalSystem<dim> sys(n);

  // natural boundary conditions
  sys.A[0][0] = 0.0F;
  sys.A[0][1] = 1.0F;
  sys.A[0][2] = 0.0F;
  sys.A[n-1][0] = 0.0F;
  sys.A[n-1][1] = 1.0F;
  sys.A[n-1][2] = 0.0F;
  for (int j = 0; j < dim; ++j) {
    sys.b[0][j] = 0.0F;
    sys.b[n-1][j] = 0.0F;
  }
  // tridiagonal submatrix
  for (int i = 1; i < n - 1; ++i) {
    sys.A[i][0] = (breaks[i] - breaks[i-1]) / 6.0F;
    sys.A[i][1] = (breaks[i+1] - breaks[i-1]) / 3.0F;
    sys.A[i][2] = (breaks[i+1] - breaks[i]) / 6.0F;
    for (int j = 0; j < dim; ++j) {
      sys.b[i][j] = (vals[i+1][j] - vals[i][j]) / (breaks[i+1] - breaks[i])
          - (vals[i][j] - vals[i-1][j]) / (breaks[i] - breaks[i-1]);
    }
  }
  // solve system
  std::vector<std::array<float,dim>>& vals2 = sys.solve();

  // compute polynomial coefficients
  coefs = std::vector<std::array<std::array<float,4>,dim>> (n - 1);
  for (int i = 0; i < n - 1; ++i) {
    float h = breaks[i+1] - breaks[i];
    for (int j = 0; j < dim; ++j)  {
      coefs[i][j][3] = (-vals2[i][j] + vals2[i+1][j]) / (6.0F * h);
      coefs[i][j][2] = 0.5F * vals2[i][j];
      coefs[i][j][1] = (-vals[i][j] + vals[i+1][j]) / h
          - h * (2.0F * vals2[i][j] + vals2[i+1][j]) / 6.0F;
      coefs[i][j][0] = vals[i][j];
    }
  }
}

void Spline::computeCoefficientsPeriodic()
{
  int n = breaks.size() - 1; // linear equation system size

  // ensure that last waypoint equals to first waypoint
  vals[0] = vals[n];

  // Linear system is for the 2 splines and the Sherman-Morrison vector u
  TridiagonalSystem<dim+1> sys(n);

  // non-tridiagonal matrix
  sys.A[0][0] = (breaks[n] - breaks[n-1]) / 6.0F;
  sys.A[0][1] = (breaks[1] - breaks[0] + breaks[n] - breaks[n-1]) / 3.0F;
  sys.A[0][2] = (breaks[1] - breaks[0]) / 6.0F;
  for (int j = 0; j < dim; ++j) {
    sys.b[0][j] = (vals[1][j] - vals[0][j]) / (breaks[1] - breaks[0])
        - (vals[n][j] - vals[n-1][j]) / (breaks[n] - breaks[n-1]);
  }
  sys.b[0][2] = 0.0F;
  for (int i = 1; i < n; ++i) {
    sys.A[i][0] = (breaks[i] - breaks[i-1]) / 6.0F;
    sys.A[i][1] = (breaks[i+1] - breaks[i-1]) / 3.0F;
    sys.A[i][2] = (breaks[i+1] - breaks[i]) / 6.0F;
    for (int j = 0; j < dim; ++j) {
      sys.b[i][j] = (vals[i+1][j] - vals[i][j]) / (breaks[i+1] - breaks[i])
          - (vals[i][j] - vals[i-1][j]) / (breaks[i] - breaks[i-1]);
    }
    sys.b[i][2] = 0.0F;
  }

  // Sherman-Morrison formula yields tridiagonal matrix
  // https://www.cfd-online.com/Wiki/Tridiagonal_matrix_algorithm_-_TDMA_(Thomas_algorithm)
  std::array<float,2> u { { -sys.A[0][1], sys.A[n-1][2] } } ;
  std::array<float,2> v { { 1.0F, -sys.A[0][0] / sys.A[0][1] } };
  sys.b[0][2] = u[0];
  sys.b[n-1][2] = u[1];
  sys.A[0][0] = 0.0F;
  sys.A[0][1] -= u[0] * v[0];
  sys.A[n-1][1] -= u[1] * v[1];
  sys.A[n-1][2] = 0.0F;

  // Solve modified system: y[:,0:1] = x[:,0:1] , q[:] = x[:,2]
  std::vector<std::array<float,dim+1>>& x = sys.solve();
  for (int j = 0; j < dim; ++j) {
    const float a { (v[0]*x[0][j] + v[1]*x[n-1][j]) / (1 + v[0]*x[0][2] + v[1]*x[n-1][2]) };
    x[0][j] -= a * x[0][2];
    x[n-1][j] -= a * x[n-1][2];
  }

  // Last point equals to first point
  x.push_back(x[0]);

  // compute polynomial coefficients
  coefs = std::vector<std::array<std::array<float,4>,dim>> (breaks.size() - 1);
  for (int i = 0; i < n; ++i) {
    float h = breaks[i+1] - breaks[i];
    for (int j = 0; j < dim; ++j)  {
      coefs[i][j][3] = (-x[i][j] + x[i+1][j]) / (6.0F * h);
      coefs[i][j][2] = 0.5F * x[i][j];
      coefs[i][j][1] = (-vals[i][j] + vals[i+1][j]) / h
          - h * (2.0F * x[i][j] + x[i+1][j]) / 6.0F;
      coefs[i][j][0] = vals[i][j];
    }
  }
}

void Spline::interpolate(float x, std::array<float, dim>& y,
                         std::array<float, dim>& yd,
                         std::array<float, dim>& ydd,
                         const int pieceIdx)
{
  int i { 0 };
  if (boundaryCondition == BoundaryCondition::periodic) {
    x = std::fmod(x, breaks.back());
  }
  if (pieceIdx >= 0) {
    i = pieceIdx;
  } else {
    // this is the default of the opional parameter pieceIdx
    i = binarySearch(x);
  }

  // evaluate polynomials for y and its derivatives
  float dx = x - breaks[i];
  for (int j = 0; j < dim; ++j) {
    y[j] = ((coefs[i][j][3] * dx + coefs[i][j][2]) * dx + coefs[i][j][1]) * dx + coefs[i][j][0];
    yd[j] = (3.0F * coefs[i][j][3] * dx + 2.0F * coefs[i][j][2]) * dx + coefs[i][j][1];
    ydd[j] = 6.0F * coefs[i][j][3] * dx + 2.0F * coefs[i][j][2];
  }
}

int Spline::getNearest(const std::array<float, dim>& y, float& x, float& dist)
{
  // compute the distances of y to the waypoints
  // and find the break with the minimal distance
  float dmin = FLT_MAX;
  int imin1 = 0;
  for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
    float d = getDistance(y, i);
    if (d < dmin) {
      dmin = d;
      imin1 = i;
    }
  }

  // find the second smallest distance
  // search the 2 neighbors of imin
  int imin2 { 0 };
  int iprev = imin1 - 1;
  int inext = imin1 + 1;
  if (boundaryCondition == BoundaryCondition::periodic) {
    if (iprev < 0) {
      iprev = breaks.size() - 2;
    }
    if (inext > static_cast<int>(breaks.size()) - 1) {
      inext = 1;
    }
  }
  if (iprev < 0 || iprev >= static_cast<int>(breaks.size())) {
    // no previous point (may be the case only for non-periodic splines)
    // take next point
    imin2 = inext;
  } else if (inext < 0 || inext >= static_cast<int>(breaks.size())) {
    // no next point (may be the case only for non-periodic splines)
    // take previous point
    // imin1 is always less than imin2
    imin2 = imin1;
    imin1 = iprev;
  } else if (getDistance(y, iprev) < getDistance(y, inext)) {
    // next point and previous points exist
    // imin1 is always less than imin2
    imin2 = imin1;
    imin1 = iprev;
  } else {
    // next point and previous points exist
    imin2 = inext;
  }

  // interpolate linearly between the 2 neighbors
  // vectorial difference
  const std::array<float, dim> y12 { { vals.at(imin2).at(0) - vals.at(imin1).at(0) ,
          vals.at(imin2).at(1) - vals.at(imin1).at(1) } };
  const std::array<float, dim> y1 { { y.at(0) - vals.at(imin1).at(0) ,
          y.at(1) - vals.at(imin1).at(1) } };
  float x12 = breaks.at(imin2) - breaks.at(imin1);

  // fix overruns in case of periodic splines
  if (boundaryCondition == BoundaryCondition::periodic) {
    if (x12 > breaks.back()) {
      x12 = x12 - breaks.back();
    } else if (x12 < 0.0F) {
      x12 = x12 + breaks.back();
    }
  }

  // distance of y to imin1 by dot product
  float dx = (y12.at(0) * y1.at(0) + y12.at(1) * y1.at(1)) * x12
      / (y12.at(0)*y12.at(0) + y12.at(1)*y12.at(1));

  // limit dx
  if (dx < 0.0F) {
    dx = 0.0F;
  } else if (dx > x12) {
    dx = x12;
  }
  const float xinit = breaks.at(imin1) + dx;

  // Newton-Raphson
  x = xinit; // first guess
  const int itermax { 3 };
  for (int iter = 0; iter < itermax; ++iter) {
    std::array<float, dim> sv;
    std::array<float, dim> sdv;
    std::array<float, dim> sddv;
    interpolate(x, sv, sdv, sddv, imin1);
    // first derivative of distance
    const float dd = -2.0F * ((y.at(0) - sv.at(0)) * sdv.at(0)
                              + (y.at(1) - sv.at(1)) * sdv.at(1));
    // second derivative of distance
    const float ddd = 2.0F * (1.0F - (y.at(0) - sv.at(0)) * sddv.at(0)
                              - (y.at(1) - sv.at(1)) * sddv.at(1));
    // epsilon error of ddd and deltax
    const float eps { 1e-3F };
    if (std::fabs(ddd) < eps) {
      break;
    }
    // update
    const float deltax = -dd / ddd;
    x += deltax;
    if (std::fabs(deltax) < eps) {
      break;
    }
  }

  if (x < breaks.at(imin1) || x > breaks.at(imin1) + x12) {
    x = xinit;
  }

  // compute distance
  std::array<float, dim> sv;
  std::array<float, dim> sdv;
  std::array<float, dim> sddv;
  interpolate(x, sv, sdv, sddv, imin1);

  dist = std::sqrt((y.at(0) - sv.at(0)) * (y.at(0) - sv.at(0)) +
                   (y.at(1) - sv.at(1)) * (y.at(1) - sv.at(1)));
  return imin1;
}

int Spline::binarySearch(const float x)
{
  // binsearch over pieces
  int imin = 0;
  int imax = breaks.size() - 1;
  int i = 0;
  while (imin < imax - 1) {
    i = (imin + imax) / 2;
    if (x >= breaks.at(i)) {
      imin = i;
    } else if (x < breaks[i]) {
      imax = i;
    }
  }
  return imin;
}

void Spline::getValVector(std::vector<float>& val, int32_t idx)
{
  val.resize(breaks.size());
  for (int32_t i = 0; i < static_cast<int>(breaks.size()); ++i) {
    val[i] = vals[i][idx];
  }
}

}

