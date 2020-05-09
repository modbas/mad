function track = mbc_obstacle_create(track, x, w1, w, l)
% track = mbc_obstacle_create(track, x, l, w) creates an rectangular 
% obstacle
% 
%   track - existing track created by mbc_track_create.
%           The return value track contains the original track plus
%           the new obstacle.
%   x - arc length at which obstacle is placed[ m ]
%   w1 - position of right side of obstacle from right border [ m ]
%   w - width of obstacle [ m ]
%   l - length of obstacle [ m ]
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

cnt = length(track.obstacles);
track.obstacles{cnt+1} = struct('x', x, ...
    'w1', w1, 'l', l,  'w', w);
end