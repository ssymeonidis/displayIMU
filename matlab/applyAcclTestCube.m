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

% initialize simulation
clear all; close all;

% test zero-pitch, zero-roll
euler  = [25, 15, 15];
accl   = [0, 0, 1];
run_sim(euler, accl)

% test 90deg-pitch
euler  = [15, 65, 15];
accl   = [-1, 0, 0];
run_sim(euler, accl)

% test 90deg-roll
euler  = [15, 15, 65];
accl   = [0, 1, 0];
run_sim(euler, accl)

% test 180deg-roll
euler  = [15, 15, 155];
accl   = [0, 0, -1];
run_sim(euler, accl)

% test neg90-pitch
euler  = [-15, -65, -15];
accl   = [1, 0, 0];
run_sim(euler, accl)

% test neg90-roll
euler  = [-15, -15, -65];
accl   = [0, -1, 0];
run_sim(euler, accl)


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler = run_sim(euler, accl)

  % define local constants
  alpha      = 0.005;
  iter       = 100;
  method     = "gradient";

  % convert orientation angles to quaternion
  euler_rad  = pi * euler / 180;
  q          = eulerToQuat(euler_rad);
  
  % main processing loop
  FOM    = [];
  for i=1:iter
    if     (method == "gradient")
      [q, FOM(i)] = applyAcclGradient (q, accl, alpha);
    elseif (method == "rotate")
      q           = applyAcclRotate   (q, accl, alpha);
    end
    display_state(q);
  end

  % print results
  euler_rad  = quatToEuler(q);
  euler      = 180 * euler_rad / pi;
end


%% updates the display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  u             = quatRotateForward([0, 0, 1], q, "full");
  f             = quatRotateForward([1, 0, 0], q, "full");
  r             = quatRotateForward([0, 1, 0], q, "full");
  plotVector(u, f, r);
  title('applyAcclTest');
  delete(findall(gcf,'type','annotation'));
  loc           = [.75 .67 .6 .3];
  str{1}        = 'red = up';
  str{2}        = 'green = forward';
  str{3}        = 'blue = right';
  annotation('textbox', loc, 'String', str, 'FitBoxToText', 'on');
  drawnow;
end
