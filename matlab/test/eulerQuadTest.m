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
clear all; close all;
addpath('..');
disp(' ');


% test 18
disp('#1');
disp('test: f = 1, 0, 0')
disp('      u = 0, 0, -1');
f1 = [1, 0, 0];
u1 = [0, 0, -1];
q1 = quat("upFrwd", u1, f1);% test 1
disp('test: f = 1, 0, 0')
disp('      u = 0, 0, 1');
f1 = [1, 0, 0];
u1 = [0, 0, 1];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 2
disp('#2');
disp('test: u = 0, 0, 1')
u1 = [0, 0, 1];
q1 = quat("up", u1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 3
disp('#3');
disp('test: f = 1, 0, 0')
f1 = [1, 0, 0];
q1 = quat("frwd", f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 4
disp('#4');
disp('test: f = 0, 1, 0')
disp('      u = 0, 0, 1');
f1 = [0, 1, 0];
u1 = [0, 0, 1];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 5
disp('#5');
disp('test: f = 0, 1, 0')
f1 = [0, 1, 0];
q1 = quat("frwd", f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 6
disp('#6');
disp('test: f = 0, -1, 0')
disp('      u = 0, 0, 1');
f1 = [0, -1, 0];
u1 = [0, 0, 1];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 7
disp('#7');
disp('test: f = 0, -1, 0')
f1 = [0, -1, 0];
q1 = quat("frwd", f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 8
disp('#8');
disp('test: f = -1, 0, 0')
disp('      u = 0, 0, 1');
f1 = [-1, 0, 0];
u1 = [0, 0, 1];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 9
disp('#9');
disp('test: f = -1, 0, 0')
f1 = [-1, 0, 0];
q1 = quat("frwd", f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 10
disp('#10');
disp('test: f = 0, 0, 1')
disp('      u = -1, 0, 0');
f1 = [0, 0, 1];
u1 = [-1, 0, 0];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 11
disp('#11');
disp('test: u = -1, 0, 0')
u1 = [-1, 0, 0];
q1 = quat("up", u1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 12
disp('#12');
disp('test: f = 0, 0, -1')
disp('      u = 1, 0, 0');
f1 = [0, 0, -1];
u1 = [1, 0, 0];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 13
disp('#13');
disp('test: u = 1, 0, 0')
u1 = [1, 0, 0];
q1 = quat("up", u1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 14
disp('#14');
disp('test: f = 1, 0, 0')
disp('      u = 0, 1, 0');
f1 = [1, 0, 0];
u1 = [0, 1, 0];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 15
disp('#15');
disp('test: u = 0, 1, 0')
u1 = [0, 1, 0];
q1 = quat("up", u1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 16
disp('#16');
disp('test: f = 1, 0, 0')
disp('      u = 0, -1, 0');
f1 = [1, 0, 0];
u1 = [0, -1, 0];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 17
disp('#17');
disp('test: u = 0, -1, 0')
u1 = [0, -1, 0];
q1 = quat("up", u1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 18
disp('#18');
disp('test: f = 1, 0, 0')
disp('      u = 0, 0, -1');
f1 = [1, 0, 0];
u1 = [0, 0, -1];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 19
disp('#19');
disp('test: u = 0, 0, -1')
u1 = [0, 0, -1];
q1 = quat("up", u1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);

% test 20
disp('#20');
disp('test: f = 0, 1, 0')
disp('      u = -1, 0, 0');
f1 = [0, 1, 0];
u1 = [-1, 0, 0];
q1 = quat("upFrwd", u1, f1);
a  = q1.deg;
q2 = quat("deg", a);
disp_results(q1.frwd, q1.up, q1.rght, a, q2);


%% print function
function disp_results(f, u, r, a, q)

% display input vectors
f = round(f, 3);
u = round(u, 3);
r = round(r, 3);
disp(['in:   f = ', num2str(f(1)), ', ', num2str(f(2)), ', ' num2str(f(3))]);
disp(['      u = ', num2str(u(1)), ', ', num2str(u(2)), ', ' num2str(u(3))]);
disp(['      r = ', num2str(r(1)), ', ', num2str(r(2)), ', ' num2str(r(3))]);

% display euler angles
a = round(a, 0);
disp(['ang:  y = ', num2str(a(1))]);
disp(['      p = ', num2str(a(2))]);
disp(['      r = ', num2str(a(3))]);

% display output vectors
f = round(q.frwd, 3);
u = round(q.up,   3);
r = round(q.rght, 3);
disp(['out:  f = ', num2str(f(1)), ', ', num2str(f(2)), ', ' num2str(f(3))]);
disp(['      u = ', num2str(u(1)), ', ', num2str(u(2)), ', ' num2str(u(3))]);
disp(['      r = ', num2str(r(1)), ', ', num2str(r(2)), ', ' num2str(r(3))]);
disp(' ');

end
