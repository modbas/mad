function [ w, widx ]  = mbc_spline_get_reference(s, breakslen, points, ppcoefs, ...
    periodic, vref, Tt)
%#codegen
% w  = mbc_spline_get_reference(s, breakslen, points, ppcoefs, ...
%      periodic, vref, Tt) calculates the reference signal for
%      path following control.
%
%   s - current car position: vector with x- and y-coordinates
%   breakslen - number of breaks
%   points - interpolation points on spline. 3 x n matrix.
%            points(1,:) is the discrete arc length.
%            points(2,:) are the x-coordinates (s1).
%            points(3,:) are the y-coordinates (s2).
%   ppcoefs - piecewise polynomial coefficients of path
%   periodic - true if circular track, false otherwise
%   vref - speed for lookahead
%   Tt - steering delay for lookahead of kappa
%   w - reference signal: [ arc length x, orientation angle psi [rad],
%       s1, s2, curvature kappa ]
%   widx - index nearest point in spline
%
% See also mbc_spline_get_nearest, mbc_ppval.
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

if breakslen > 0
    [ wx, imin1 ] = mbc_spline_get_nearest(breakslen, points, ppcoefs, periodic, s);
    [ wsv, wsdv, wsddv, widx ] = mbc_ppval(breakslen, points(1,:), ppcoefs, wx, imin1);
    ws1 = wsv(1);
    ws2 = wsv(2);
    wpsi = atan2(wsdv(2), wsdv(1));
    wxlookahead = wx + vref * Tt;
    if periodic == true
        if wxlookahead > points(1, breakslen)
            wxlookahead = wxlookahead - points(1, breakslen);
        elseif wx < 0
            wxlookahead = wxlookahead + points(1, breakslen);
        end
    end
    [ wsv, wsdv, wsddv, widx ] = mbc_ppval(breakslen, points(1,:), ppcoefs, wxlookahead);
    wkappa = sqrt(wsddv(1) * wsddv(1) + wsddv(2) * wsddv(2));
    wkappasign = wsdv(1) * wsddv(2) - wsddv(1) * wsdv(2);
    if wkappasign < 0
        wkappa = -wkappa;
    end
    w = [ wx; wpsi; ws1; ws2; wkappa ];
else
    w = [ 0; 0; 0; 0; 0 ];
    widx = -1;
end
