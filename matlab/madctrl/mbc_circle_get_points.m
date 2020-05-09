function points = mbc_circle_get_points(track, idx, xstart, dx, alpha)
% points = mbc_circle_get_points(track, idx, xstart, dx, alpha)
% computes the accurate line points on the track segment.
%
%   track - track object
%   idx - the index of the track segment
%   xstart - the arc length of the segment start positon [ m ]
%   dx - the arc distance of two neighbor points [ m ]
%   alpha - the line position on the road [ 0 ; 1 ]
%   points - the points as a 3 x n matrix
%
%   points(1, :) is the arc length at each point
%   points(2, :) is the s1 position
%   points(3, :) is the s2 position
%
%   If alpha == 0 then the right line points are generated.
%   If alpha == 0.5 then the center line points are generated.
%   If alpha == 1 then the left line points are generated.
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

if nargin < 4
    alpha = 0.5; % return center line points
end
p = track.points{idx};
t = track.tracks{idx};
r = t.r - sign(t.xe) * (alpha - 0.5) * t.w;
center1 = p.s1 - sign(t.xe) * t.r * cos(p.psi - pi/2);
center2 = p.s2 - sign(t.xe) * t.r * sin(p.psi - pi/2);
idx = 0:floor((abs(t.xe) - (xstart - p.x) + mbc_cmp_eps)/dx);
x = (xstart - p.x) + dx * idx;
points = [ p.x + x ; ...
    center1 + sign(t.xe) * r * cos(p.psi - pi/2 + sign(t.xe) * x / t.r) ; ...
    center2 + sign(t.xe) * r * sin(p.psi - pi/2 + sign(t.xe) * x / t.r) ];
end