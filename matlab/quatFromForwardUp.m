% This file is part of quaternion-based displayIMU C++/QT code base
% (https://github.com/ssymeonidis/displayIMU.git)
% Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
% Time (SMRT) Processing Solutions)
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

function q = quatFromForwardUp(f,u)

% normalize reference (up) vector
u = u./norm(u);

% ortho normalize forward vector 
D = sum(f.*u);
f = f - D*u; 
f = f./norm(f);

% calculate right vector
r = cross(u,f);

% calcuate the quaternion
M = [f(1), r(1), u(1); ...
     f(2), r(2), u(2); ...
     f(3), r(3), u(3)];
q = matrixToQuat(M);

end
