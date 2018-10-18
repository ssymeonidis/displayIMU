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
% Multipling two quaternions
function q = quatMultiply(in1,in2)
  q(1)  = in2(1)*in1(1) - in2(2)*in1(2) - in2(3)*in1(3) - in2(4)*in1(4);
  q(2)  = in2(1)*in1(2) + in2(2)*in1(1) - in2(3)*in1(4) + in2(4)*in1(3);
  q(3)  = in2(1)*in1(3) + in2(2)*in1(4) + in2(3)*in1(1) - in2(4)*in1(2);
  q(4)  = in2(1)*in1(4) - in2(2)*in1(3) + in2(3)*in1(2) + in2(4)*in1(1);
end
