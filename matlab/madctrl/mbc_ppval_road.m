function y = mbc_ppval_road(ppbreaks, ppcoefs, x, pidx)
%#codegen
% y = mbc_ppval(ppbreaks, ppcoefs, x, pidx) embedded version of ppval.
% Used in Simulink code generated for the car.
%
% See also ppval.
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

c = zeros(2,4,'single');
[row,col]=size(ppcoefs);

if nargin < 4
    pidx = mbc_binsearch(ppbreaks, x);
end
%c = ppcoefs((2*pidx-1):(2*pidx), :);
c(1,1:col) = ppcoefs((2*pidx-1),1:col);
c(2,1:col) = ppcoefs((2*pidx),  1:col);

dx = x - ppbreaks(pidx);
y = c(:,1);
%for cidx = 2:length(c)
for cidx = 2:col
    y = y * dx + c(:,cidx);
end
end