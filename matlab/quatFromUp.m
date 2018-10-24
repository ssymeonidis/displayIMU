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

%%
function q = quatFromUp(u, method)

% check number of arguments
if (nargin < 2)
  method = "full";
end

% uses quaternion base operations
if (method == "full")
  q = quatFromTwoVectors([0, 0, 1], u);
    
% fully expanded and optimized
elseif (method == "optimized")
  err   = 0.001;
  norm  = sqrt(u(1)*u(1) + u(2)*u(2) + u(3)*u(3));
  real  = norm + u(3);
  if (real > err * norm)
    q   = [real, -u(2), u(1), 0];
    q   = q / sqrt(sum(q.^2)); 
  else
    q   = [1, 0, 0, 0];
  end

else
  error("invalid method");
end
 
end