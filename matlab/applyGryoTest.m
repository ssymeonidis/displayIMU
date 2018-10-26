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

% rotate test #1
euler      = [0, 0, 0];
axis       = [1, 0, 0];
range      = 90;              % deg
speed      = 15;              % deg/sec
euler      = run_sim(euler, axis, range, speed);

% rotate test #2
axis       = [-1, 0, 0];
range      = 90;              % deg
speed      = 15;              % deg/sec
euler      = run_sim(euler, axis, range, speed);

% rotate test #3
axis       = [0, 1, 0];
range      = 90;              % deg
speed      = 15;              % deg/sec
euler      = run_sim(euler, axis, range, speed);

% rotate test #4
axis       = [0, -1, 0];
range      = 90;              % deg
speed      = 15;              % deg/sec
euler      = run_sim(euler, axis, range, speed);

% rotate test #5
axis       = [0, 0, 1];
range      = 90;              % deg
speed      = 15;              % deg/sec
euler      = run_sim(euler, axis, range, speed);

% rotate test #6
axis       = [0, 0, -1];
range      = 90;              % deg
speed      = 15;              % deg/sec
euler      = run_sim(euler, axis, range, speed);


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler = run_sim(euler, axis, range, speed)

  % define local constants
  dt           = 0.1;
  time         = range / speed;
  iter         = time  / dt
  speed_rad    = pi * speed / 180;
  gyro         = axis * speed_rad
  
  % main processing loop
  euler_rad    = pi * euler / 180;
  q            = eulerToQuat(euler_rad);
  for i=1:iter
    q          = applyGyroIntegrate(q, gyro, dt);
    display_state(q);
  end

  % return final state
  q
  euler_rad  = quatToEuler(q);
  euler      = 180 * euler_rad / pi;
end

%% update the display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  plotState(q);
  title('applyGyroTest');
  drawnow;
end