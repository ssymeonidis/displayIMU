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
% ASSUMPTION - normalized input quaternion (use quatNormalize function)

function out = quatRotateVector(in, q, method)

% check number of arguments
if (nargin < 3)
  method = "full";
end

if     (method == "full")
  in       = [0, in];
  tmp      = quatMultiply(in,quatConjugate(q));
  out      = quatMultiply(q,tmp);
  out      = out(2:4);

elseif (method == "better")
  u        = q(2:4);
  s        = q(1);
  out      =  2.0 * dot(u, in) * u    ...
           + (s*s - dot(u, u)) * in   ...
           + 2.0 * s * cross(u, in);

else
  error("invalid method");
end

end
