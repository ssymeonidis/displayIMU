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

function q = quatFromTwoVectors(u, v, method)

if (method == 1)
  u = u./sqrt(sum(u.^2)); 
  v = v./sqrt(sum(v.^2)); 
  m = sqrt(2.0 + 2.0 * dot(u, v));
  w = (1.0 / m) * cross(u, v);
  q = [0.5 * m, w(1), w(2), w(3)];

elseif (method == 2) 
  err  = 0.00001;
  norm = sqrt(dot(u, u) * dot(v, v));
  real = norm + dot(u, v);
  if (real < err * norm)
    if abs(u(1)) > abs(u(3))
      w = [-u(2), u(1), 0];
    else
      w = [0, -u(3), u(2)];
    end
  else
    w = cross(u, v);
  end
  q = [real, w];
  q = q / sqrt(sum(q.^2));
end
  
end
