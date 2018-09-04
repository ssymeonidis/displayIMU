% This file is part of quaternion-based displayIMU C++/QT code base
% (https://github.com/ssymeonidis/displayIMU.git)
% Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
% Time (SMRT) Processing Solutions
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, version 2.
%
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

%%
% convert quaternion to matrix
function M = quatToMatrix(q)

q = quatNormalize(q);
w = q(1);
x = q(2);
y = q(3);
z = q(4);

M = [1-2*y*y-2*z*z,    2*x*y-2*w*z,    2*x*z+2*w*y;
       2*x*y+2*w*z,  1-2*x*x-2*z*z,    2*y*z-2*w*x;
       2*x*z-2*w*y,    2*y*z+2*w*x,  1-2*x*x-2*y*y]; 
