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
function q = eulerToQuat(E)

% angle function abreviations
cy = cos(E(1) * 0.5);
sy = sin(E(1) * 0.5);
cr = cos(E(3) * 0.5);
sr = sin(E(3) * 0.5);
cp = cos(E(2) * 0.5);
sp = sin(E(2) * 0.5);

% conversion to quaternion
q(1) = cy*cr*cp + sy*sr*sp;
q(2) = cy*sr*cp - sy*cr*sp;
q(3) = cy*cr*sp + sy*sr*cp;
q(4) = sy*cr*cp - cy*sr*sp;

end