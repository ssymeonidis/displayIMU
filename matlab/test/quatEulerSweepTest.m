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

% last run on 11/25/19

% initialize simulation
addpath('..');
addpath('../utils');

% define simulation inputs
step_size_deg   = 2;

% test yaw
for i=0:step_size_deg:180
  process_datum([i, 0, 0], "positive yaw");
end
for i=180:-step_size_deg:0
  process_datum([i, 0, 0], "negative yaw");
end

% test pitch
for i=0:step_size_deg:90
  process_datum([0, i, 0], "positive pitch");
end
for i=90:-step_size_deg:0
  process_datum([0, i, 0], "negative pitch");
end

% test roll
for i=0:step_size_deg:90
  process_datum([0, 0, i], "positive roll");
end
for i=90:-step_size_deg:0
  process_datum([0, 0, i], "negative roll");
end


%% main function (performs conversion and generates plot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function process_datum(euler_deg, name)
  plotState(quat("eulerDeg", euler_deg)); title(name);
  drawnow;
end