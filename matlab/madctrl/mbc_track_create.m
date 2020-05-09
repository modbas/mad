function this = mbc_track_create(s1, s2, psi)
% this = mbc_track_create(s1, s2, psi) creates a new empty track.
%   s1 - initial x-coordinate
%   s2 - initial y-coordinate
%   psi - initial yaw angle
%   this - the new track
%
% See also mbc_straight_create, mbc_circle_create
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

this = struct('points', cell(1), ...
    'tracks', cell(1), ...
    'obstacles', cell(1));
this.points{1} = struct('s1', s1, ...
    's2', s2, 'psi', psi, 'x', 0);
end