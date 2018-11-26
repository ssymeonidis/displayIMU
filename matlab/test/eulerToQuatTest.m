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

%% generate input sweeps for unit under test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize simulation
addpath('..');
addpath('../utils');

% define simulation inputs
step_size_deg   = 2;

% test yaw(positive sweep)
for i=0:step_size_deg:180
  process_datum([i, 0, 0]);
end
for i=180:-step_size_deg:0
  process_datum([i, 0, 0]);
end

% test pitch (positive sweep)
for i=0:step_size_deg:90
  process_datum([0, i, 0]);
end
for i=90:-step_size_deg:0
  process_datum([0, i, 0]);
end

% test roll (positive sweep)
for i=0:step_size_deg:90
  process_datum([0, 0, i]);
end
for i=90:-step_size_deg:0
  process_datum([0, 0, i]);
end


%% main function (performs conversion and generates plot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function process_datum(euler_deg)
  plotState(quat("deg", euler_deg));
  drawnow;
end