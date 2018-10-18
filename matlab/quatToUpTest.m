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

% define simulation inputs
method          = "optimized";
step_size_deg   = 2;

% test pitch (positive sweep)
for i=0:step_size_deg:90
  process_datum([0, i, 0], method);
end
for i=90:-step_size_deg:0
  process_datum([0, i, 0], method);
end

% test roll(positive sweep)
for i=0:step_size_deg:90
  process_datum([0, 0, i], method);
end
for i=90:-step_size_deg:0
  process_datum([0, 0, i], method);
end

%% main function (performs conversion and generates plot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function process_datum(euler_deg, method)
  q             = eulerToQuat(pi*euler_deg/180);
  u             = quatToUp(q, method);
  plotVector(u);
  delete(findall(gcf,'type','annotation'))
  title('quatToUpTest');
  str{1}        = sprintf('y = %0.2f deg', euler_deg(1));
  str{2}        = sprintf('p = %0.2f deg', euler_deg(2));
  str{3}        = sprintf('r = %0.2f deg', euler_deg(3));
  loc           = [.02 .67 .6 .3];
  annotation('textbox', loc, 'String', str, 'FitBoxToText', 'on');
  drawnow;
end