function [ x, imin1 ] = mbc_spline_get_nearest(breakslen, splinepoints, ppcoefs, splineperiodic, s)
%#codegen
% [ x, imin1 ] = mbc_spline_get_nearest(breakslen, splinepoints, ppcoefs, splineperiodic, s) calculates
% the arc length of the spline point nearest to the point s.
%
%   breakslen - number of points
%   splinepoints - interpolation points on spline. 3 x n matrix.
%                  splinepoints(1,:) are the arc lengths.
%                  splinepoints(2,:) are the x-positions
%                  splinepoints(3,:) are the y-positions
%   ppcoefs - piecewise polynomial coefficients of path
%   splineperiodic - true if spline is periodic (point(1) == point(n))
%   s - point vector with x position and y position
%   x - arc length of point on spline nearest to s
%   imin1 - index of spline break with
%           s(imin1) <= s < s(imin1+1) where s the arc length
%
% mbc_spline_get_nearest computes the arc length of the reference point for
% path following control from the current position s. It is called
% by mbc_spline_get_reference.
%
% See also mbc_spline_get_reference
%
% MODBAS CAR mbc
% Copyright (c) 2020 Frank Traenkle
% http://www.modbas.de
%
% This file is part of Mini-Auto-Drive.
%
% Mini-Auto-Drive is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% Mini-Auto-Drive is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.

n = double(breakslen);

%% Find the smallest distance
dmin = inf;
imin1 = 0;

siz = size(splinepoints);
d = zeros(1, siz(2));
for i = 1:n
    d(i) = (splinepoints(2, i) - s(1)) ^ 2 + (splinepoints(3, i) - s(2)) ^ 2;
    if d(i) < dmin
        dmin = d(i);
        imin1 = i;
    end
end

%% Find the second smallest distance
%% Search the 2 neighbors of imin
iprev = imin1 - 1;
inext = imin1 + 1;
if splineperiodic
    if iprev < 1
        %% note: if periodic spline: point(1) == point(n)
        iprev = n - 1;
    end
    if inext > n
        %% note: if periodic spline: point(1) == point(n)
        inext = 2;
    end
end
if iprev < 1 || iprev > n
    % no previous point (may be the case only for non-periodic splines)
    % take next point
    imin2 = inext;
elseif inext < 1 || inext > n
    % no next point (may be the case only for non-periodic splines)
    % take previous point
    % imin1 is always less than imin2
    imin2 = imin1;
    imin1 = iprev;
elseif d(iprev) < d(inext)
    % next and previous points exist
    % imin1 is always less than imin2
    imin2 = imin1; 
    imin1 = iprev; 
else
    % next point and previous points exist
    imin2 = inext;
end

%% Interpolate linearly between 2 neighbors
d12 = splinepoints(2:3, imin2) - splinepoints(2:3, imin1);
d1 = s - splinepoints(2:3, imin1);
x12 = splinepoints(1, imin2) - splinepoints(1, imin1);

%% fix overruns in case of periodic splines
if splineperiodic == true
    if x12 > splinepoints(1, breakslen)
        x12 = x12 - splinepoints(1, breakslen);
    elseif x12 < 0
        x12 = x12 + splinepoints(1, breakslen);
    end
end

%% first guess
dx = (d12' * d1) * x12 / (d12' * d12);
if dx < 0
    dx = dx - dx; % set to zero (works both for single or double dx)
elseif dx > x12
    dx = x12;
end
xinit = splinepoints(1, imin1) + dx;

%% Newton-Raphson
x = xinit; % init guess
for iter = 1:3
    [ sv, sdv, sddv, xidx ] = mbc_ppval(breakslen, splinepoints(1,:), ppcoefs, x, imin1);
    dd = -2 * ((s(1) - sv(1)) * sdv(1) + (s(2) - sv(2)) * sdv(2)); % first derivative of distance
    ddd = 2 * (1 - (s(1) - sv(1)) * sddv(1) - (s(2) - sv(2)) * sddv(2)); % second derivative of distance
    if abs(ddd) < 1e-3
        break;
    end
    deltax = -dd / ddd;    
    x = x + deltax;
    if abs(deltax) < 1e-3
        break;
    end
end
if x < splinepoints(1, imin1) || x > splinepoints(1, imin1) + x12
    x = xinit;
end
end
