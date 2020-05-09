function ret = mbc_carerror_display(s, P, scale)
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

persistent axes;
persistent ellipse;
persistent unitCirclePoints;

if isempty(axes)
    figure(1);
    phi = linspace(0, 2*pi, 36);
    unitCirclePoints = [ cos(phi) ; sin(phi) ];
    color = 'r';
    width = 3;
    axes = [ line([ -0.1 0.1 ], [ 0 0 ], 'Color', color, 'LineWidth', width), ...
             line([ 0 0 ], [ -0.1 0.1 ], 'Color', color, 'LineWidth', width) ];
    ellipse = line(unitCirclePoints(1,:), unitCirclePoints(2,:), 'Color', color, 'LineWidth', width);
end

[ v, e ] = eig(P);
v = real(v);
e = real(e);
T = v * e * scale;
ellipsePoints =  T * unitCirclePoints ... 
     + [ s(1,1) * ones(1, length(unitCirclePoints)) ; s(2,2) * ones(1, length(unitCirclePoints)) ];
set(ellipse, 'XData', ellipsePoints(1,:), ...
             'YData', ellipsePoints(2,:))
s1 = s - T;
s2 = s + T;
for i = 1:2
    set(axes(i), 'XData', [ s1(1,i); s2(1,i) ], ...
                 'YData', [ s1(2,i); s2(2,i) ] );
end

ret = [ e(1,1); e(2,2) ];
end