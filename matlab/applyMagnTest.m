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

% define simulation inputs/constants
q      = [0.7, 0.3, 0.3, 0.0];
q      = q / sqrt(sum(q.^2));
magn   = [1, 0, 0];
alpha  = 0.01;
iter   = 70;

% apply current acceleration
FOM    = [];
for i=1:iter
 [q, FOM(i)] = applyMagnGradient(q, magn, alpha);
  display_state(q);
end

% graph results
figure;
plot(FOM);


%% main function (performs conversion and generates plot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  u             = quatRotate([0, 0, 1], q, "full");
  f             = quatRotate([1, 0, 0], q, "full");
  r             = quatRotate([0, 1, 0], q, "full");
  plotVector(u, f, r);
  title('eulerToQuatTest');
  delete(findall(gcf,'type','annotation'));
  loc           = [.75 .67 .6 .3];
  str{1}        = 'red = up';
  str{2}        = 'green = forward';
  str{3}        = 'blue = right';
  annotation('textbox', loc, 'String', str, 'FitBoxToText', 'on');
  drawnow;
end