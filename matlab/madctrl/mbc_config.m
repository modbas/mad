function config = mbc_config()
% config = mbc_config() returns config data for MODBAS CAR.
%   config - struct containing config data
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

config.track.debug = 1;
config.track.color = 'k';
config.track.centercolor = 'w';
config.track.centerwidth = 1;
config.track.centerstyle = '--';
config.track.bgcolor = zeros(1, 3);
config.track.bordercolor = 'w';
config.track.borderwidth = 2;
config.track.borderstyle = '-';
config.figure.bgcolor = 0.3 * ones(1, 3);
config.car.colors = { 'y', 'r', 'b' };
config.carwheel.color = 'g';
config.carwheel.w = 0.02;
config.carwheel.l = 0.1;
config.carvelocity.color = 'w';
config.refvelocity.color = 'r';
config.ir.color = 'r';
config.cartrace.size = 0.02;
config.cartrace.color = [ 1 1 1 ];
config.cartrace.length = 40;
config.cartrace.dt = 20e-3;
config.carerror.color = 'w';
config.obstacle.color = 'w';
end