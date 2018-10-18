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

% define simulation inputs/constants
q      = [0.7, 0.3, 0.3, 0.0];
q      = q / sqrt(sum(q.^2));
accl   = [0, 0, 1];
alpha  = 0.01;
iter   = 70;

% apply current acceleration
FOM    = [];
for i=1:iter
 [q, FOM(i)] = applyAcclGradient(q, accl, alpha);
end

% graph results
plot(FOM);