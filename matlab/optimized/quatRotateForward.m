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

function v_out = quatRotateForward(v, q)

v_out    = [2 * (v(1)*(0.5 - q(3)*q(3) - q(4)*q(4))   ...
               + v(2)*(q(2)*q(3) - q(1)*q(4))         ...
               + v(3)*(q(2)*q(4) + q(1)*q(3))),       ...
                
            2 * (v(1)*(q(2)*q(3) + q(1)*q(4))         ...
               + v(2)*(0.5 - q(2)*q(2) - q(4)*q(4))   ...
               + v(3)*(q(3)*q(4) - q(1)*q(2))),       ...
                
            2 * (v(1)*(q(2)*q(4) - q(1)*q(3))         ...
               + v(2)*(q(3)*q(4) + q(1)*q(2))         ...
               + v(3)*(0.5 - q(2)*q(2) - q(3)*q(3)))]';

end