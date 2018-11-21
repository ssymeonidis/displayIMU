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

function out = quatToUp(q)

out(1)  = 2 * (q(2)*q(4) + q(1)*q(3));
out(2)  = 2 * (q(3)*q(4) - q(1)*q(2));
out(3)  = 2 * (0.5 - q(2)*q(2) - q(3)*q(3));

end