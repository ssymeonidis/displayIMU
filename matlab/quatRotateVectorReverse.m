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
% The rotates this quaterion, representing a unit vector, by the input
% quaternion
function out = quatRotateReverse(in,q)
  method     = 2;
  if (method == 0)
    in       = [0, in];
    q        = quatNormalize(q);
    tmp      = quatMultiply(in,q);
    out      = quatMultiply(quatConjugate(q),tmp);
    out      = out(2:4);
  else
    out      = [2*in(1)*(0.5-q(3)*q(3)-q(4)*q(4)) + ...
                2*in(2)*(q(1)*q(4)+q(2)*q(3))     + ...
                2*in(3)*(q(2)*q(4)-q(1)*q(3))     , ...
                2*in(1)*(q(2)*q(3)-q(1)*q(4))     + ...
                2*in(2)*(0.5-q(2)*q(2)-q(4)*q(4)) + ...
                2*in(3)*(q(1)*q(2)+q(3)*q(4))     , ...
                2*in(1)*(q(1)*q(3)+q(2)*q(4))     + ...
                2*in(2)*(q(3)*q(4)-q(1)*q(2))     + ...
                2*in(3)*(0.5-q(2)*q(2)-q(3)*q(3))];
  end 
end

