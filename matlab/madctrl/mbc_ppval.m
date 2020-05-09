function [ y, yd, ydd, pidx ] = mbc_ppval(varargin)
%#codegen
% y = mbc_ppval(pp, x)
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

if nargin == 2
    ppbreaks = varargin{1}.breaks;
    breakslen = length(ppbreaks);
    ppcoefs = varargin{1}.coefs;
    xarr = varargin{2};
else
    breakslen = varargin{1};
    ppbreaks = varargin{2};
    ppcoefs = varargin{3};
    xarr = varargin{4};
end
if nargin == 5
    pidx = varargin{5};
else
    pidx = mbc_binsearch(breakslen, ppbreaks, xarr(1));
end
y = zeros(2, length(xarr));
yd = zeros(2, length(xarr));
ydd = zeros(2, length(xarr));
for i = 1:length(xarr)
    x = xarr(i);
    if pidx < breakslen && x >= ppbreaks(pidx+1)
        pidx = pidx + 1;
    end
    c = ppcoefs((2*pidx-1):(2*pidx), :);
    dx = x - ppbreaks(pidx);
    y(:,i) = c(:,1);
    for cidx = 2:length(c)
        y(:,i) = y(:,i) * dx + c(:,cidx);
    end
    yd(:,i) = (3 * c(:,1) * dx + 2 * c(:,2)) * dx + c(:,3);
    ydd(:,i) = 6 * c(:,1) * dx + 2 * c(:,2);
end
end