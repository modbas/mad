function bidx = mbc_binsearch(breakslen, ppbreaks, x)
%#codegen
% bidx = mbc_binsearch(breaks, x) searches for closest match of x in breaks
%   breakslen - number of breaks
%   breaks - are the breaks of a piecewise polynomial
%   x - is the arc length
%   bidx - is the idx of the breaks element with
%          breaks(bidx) <= x < breaks(bidx+1)
% 
%   mbc_binsearch performs a binary search and is called by mbc_ppval.
%
% See also mbc_ppval.
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

bidx1 = 1;
bidx2 = double(breakslen);
while bidx1 < bidx2 - 1
    bidx = floor((bidx1 + bidx2) / 2);
    if x >= ppbreaks(bidx)
        bidx1 = bidx;
    elseif x < ppbreaks(bidx)
        bidx2 = bidx;
    end
end
bidx = bidx1;
end