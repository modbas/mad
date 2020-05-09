function this = mbc_track_display(this, dx, area)
% this = mbc_track_display(this, dx, area) calculates the splines of
% a track and displays the track.
%
%   this - the track. The return value is the original track plus the
%          new splines for center line, left line and right line.
%   dx - the step size of the arc length
%   area - the display area (argument of axis)
%
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

xe = this.points{end}.x; % Total arc length
scnt = floor(xe / dx + 0.5); % number of steps
this.dx = xe / scnt; % new step size

if mbc_cmp(this.points{1}.s1, this.points{end}.s1) ...
        && mbc_cmp(this.points{1}.s2, this.points{end}.s2)
    % We have a circular track.
    this.periodic = true;
else
    this.periodic = false;
end

this.center = mbc_spline_create(this, this.dx, 0.5);
this.right = mbc_spline_create(this, this.dx, 0);
this.left = mbc_spline_create(this, this.dx, 1);

config = mbc_config();
figure(1);
clf;
set(gca, 'Color', config.figure.bgcolor);
hold on;
grid on;

right = fnplt(this.right.pp, config.track.bordercolor, config.track.borderwidth);
center = fnplt(this.center.pp, config.track.centercolor, config.track.centerwidth);
left = fnplt(this.left.pp, config.track.bordercolor, config.track.borderwidth);

patch([ right(1,:) fliplr(left(1,:)) ]', [ right(2,:) fliplr(left(2,:)) ]', ...
    'g', ...
    'FaceColor', config.track.bgcolor, 'EdgeColor', config.track.bgcolor);
line(center(1,:), center(2,:), ...
    'Color', config.track.centercolor, ...
    'LineWidth', config.track.centerwidth, ...
    'LineStyle', config.track.centerstyle);
line(right(1,:), right(2,:), ...
    'Color', config.track.bordercolor, ...
    'LineWidth', config.track.borderwidth, ...
    'LineStyle', config.track.borderstyle);
line(left(1,:), left(2,:), ...
    'Color', config.track.bordercolor, ...
    'LineWidth', config.track.borderwidth, ...
    'LineStyle', config.track.borderstyle);
if config.track.debug
    % center line
    plot(this.center.points(2,:), this.center.points(3,:), ...
        'Color', config.track.centercolor, ...
        'Marker', 'o');
    % left border
    plot(this.left.points(2,:), this.left.points(3,:), ...
        'Color', config.track.bordercolor, ...
        'Marker', 'o');
    % right border
    plot(this.right.points(2,:), this.right.points(3,:), ...
        'Color', config.track.bordercolor, ...
        'Marker', 'o');
end

% draw obstacles
for idx = 1:length(this.obstacles)
    o = this.obstacles{idx};
    sr1 = ppval(this.right.pp, o.x); % right border
    st1 = ppval(this.right.ppd, o.x); % tangential vector
    st1 = st1 / norm(st1);
    sn1 = ppval(this.left.pp, o.x) - sr1; % normal vector
    sn1 = sn1 / norm(sn1);
    s1 = sr1 + sn1 * o.w1;
    s4 = sr1 + sn1 * (o.w1 + o.w);
    s2 = s1 + st1 * o.l;
    s3 = s4 + st1 * o.l;
    patch([ s1(1); s2(1); s3(1); s4(1) ], [ s1(2); s2(2); s3(2); s4(2) ], ...
        [ 0; 0; 0; 0 ], ...
        'FaceColor', config.obstacle.color, 'EdgeColor', config.obstacle.color);
end
    
% scale axes
axis(area);
axis equal;
grid on;
end