function track = mbc_straight_create(track, xe, w)
% track = mbc_straight_create(track, xe, w) adds a straight line 
% as a new segment to track
% 
%   track - existing track created by mbc_track_create.
%           The return value track contains the original track plus
%           the new segment.
%   xe - arc length of straight line [ m ]
%   w - track width [ m ]
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

cnt = mbc_track_get_cnt(track);
p = track.points{cnt+1};
track.points{cnt+2} = ...
    struct('s1', p.s1 + xe * cos(p.psi), ...
    's2', p.s2 + xe * sin(p.psi), ...
    'psi', p.psi, ...
    'x', p.x + xe);
track.tracks{cnt+1} = struct('type', 'straight', ...
    'xe', xe, 'w', w);
end