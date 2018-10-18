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
function plotVector(varargin)

% internal constants
grid_count = 9;
max_val    = 1;
color      = {'red', 'blue', 'green'};

% clear plot area
newplot;
hold on;

% draw x,z grid plane
spacing    = 2*max_val/(grid_count-1);
for i=-max_val:spacing:max_val
  plot3([-max_val, max_val], [i, i], [0, 0], 'Color', [0.1, 0.1, 0.1]);
  plot3([i, i], [-max_val, max_val], [0, 0], 'Color', [0.1, 0.1, 0.1]);
end

% draw vector(s)
for i = 1:length(varargin)
  val = varargin{i};
  plot3([0, val(1)], [0, val(2)], [0, val(3)], 'LineWidth', 5, 'Color', color{i});
  plot3([0, val(1)], [0, val(2)], [0, 0],      'LineWidth', 2, 'Color', color{i});
end
hold off;

% ensure "zero" will be center of plot
axis([-max_val, max_val, -max_val, max_val, -max_val, max_val]);
set(gca,'Ydir','reverse');

% label each axis
xlabel('x (positive = forward)');
ylabel('y (positive = right)');
zlabel('z (positive = up)');

end