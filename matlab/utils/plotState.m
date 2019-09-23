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

function plotState(q)

up            = q.up;
frwd          = q.frwd;
rght          = q.rght;
plotVector(up, frwd, rght);
delete(findall(gcf,'type','annotation'));
loc           = [.75 .67 .6 .3];
str{1}        = 'red = up';
str{2}        = 'green = forward';
str{3}        = 'blue = right';
annotation('textbox', loc, 'String', str, 'FitBoxToText', 'on');

end