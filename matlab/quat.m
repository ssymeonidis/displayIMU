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

classdef quat
    
properties
  val                     % quaternion value
end

properties (Constant)
  epsilon   = 0.00001;    % near zero threshold
  magThresh = 0.35;       % threshod to use "right" vector
end

methods


%% quaternion constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q   = quat(arg1, arg2, arg3, arg4, arg5)
  % default argument
  if (nargin < 1)
    q.val    = [1, 0, 0, 0];

  % numeric argument
  elseif isnumeric(arg1)
    if     ((nargin == 1) && length(arg1) == 4)
      q.val = arg1;
    elseif ((nargin == 1) && length(arg1) == 3)
      q.val = [0, arg1(1), arg1(2), arg1(3)];
    elseif (nargin == 3)
      q.val = [0, arg1, arg2, arg3];
    elseif (nargin == 4)
      q.val = [arg1, arg2, arg3, arg4];
    else
      error("invalid numeric argument(s)");
    end

  % random quaternion (Matlab only)
  elseif strcmp(arg1, "rand")
    q       = ~quat(2*rand(1,4)-1);
  
  % forward vector
  elseif strcmp(arg1, "frwd")
    if     (nargin == 2)
      q     = q.fromFrwdSafe(arg2);
    elseif (nargin == 4)
      q     = q.fromFrwdSafe([arg2, arg3, arg4]);
    else
      error("invalid numeric argument(s)");
    end
  elseif strcmp(arg1, "frwdFast")
    if     (nargin == 2)
      q     = q.fromFrwdFast(arg2);
    elseif (nargin == 4)
      q     = q.fromFrwdFast([arg2, arg3, arg4]);
    else
      error("invalid numeric argument(s)");
    end
  
  % up vector
  elseif strcmp(arg1, "up")
    if     (nargin == 2)
      q     = q.fromUpSafe(arg2);
    elseif (nargin == 4)
      q     = q.fromUpSafe([arg2, arg3, arg4]);
    else
      error("invalid numeric argument(s)");
    end
  elseif strcmp(arg1, "upFast")
    if     (nargin == 2)
      q     = q.fromUpFast(arg2);
    elseif (nargin == 4)
      q     = q.fromUpFast([arg2, arg3, arg4]);
    else
      error("invalid numeric argument(s)");
    end
  
  % forward-up vector
  elseif strcmp(arg1, "frwdUp")
    if     (nargin == 3)
      q     = q.fromFrwdUp(arg2, arg3);
    elseif (nargin == 4)
      q     = q.fromFrwdUp(arg2, arg3, arg4);
    else
      error("invalid numeric argument(s)");
    end
  elseif strcmp(arg1, "frwdUpRght")
    if     (nargin == 4)
      q     = q.fromFrwdUpRght(arg2, arg3, arg4);
    else
      error("invalid numeric argument(s)");
    end
  
  % up-forward vector
  elseif strcmp(arg1, "upFrwd")
    if     (nargin == 3)
      q     = q.fromUpFrwd(arg2, arg3);
    elseif (nargin == 4)
      q     = q.fromUpFrwd(arg2, arg3, arg4);
    else
      error("invalid numeric argument(s)");
    end
  elseif strcmp(arg1, "upFrwdRght")
    if     (nargin == 4)
      q     = q.fromUpFrwdRght(arg2, arg3, arg4);
    else
      error("invalid numeric argument(s)");
    end

  % rotation between two vectors
  elseif strcmp(arg1, "diffVect")
    if     (nargin == 3)
      q     = q.fromVectors(arg2, arg3);
    else
      error("invalid numeric argument(s)");
    end
    
  % euler angles
  elseif strcmp(arg1, "eulerRad")
    if     (nargin == 2)
      q     = q.fromEuler(arg2);
    elseif (nargin == 4)
      q     = q.fromEuler([arg2, arg3, arg4]);
    else
      error("invalid numeric argument(s)");
    end
  elseif strcmp(arg1, "eulerDeg")
    if     (nargin == 2)
      q     = q.fromEuler(pi * arg2 / 180);
    elseif (nargin == 4)
      q     = q.fromEuler(pi * [arg2, arg3, arg4] / 180);
    else
      error("invalid numeric argument(s)"); 
    end
  
  % axis angle to quaternion
  elseif strcmp(arg1, "axisAngle")
    if     (nargin == 2)
      q     = q.fromAxisAngle(arg2);
    elseif (nargin == 5)
      q     = q.fromAxisAngle([arg2, arg3, arg4, arg5]);
    end
  
  % rotation matrix
  elseif strcmp(arg1, "matrix")
    q       = q.fromMatrix(arg2);
    
  % error message
  else
    error("invalid argument");
  end
end


%% quaternion subscript reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function val = subsref(q, S)
  if       strcmp(S(1).type, '.')
    % basic operations
    if     strcmp(S(1).subs, 'val')
      val      = q.val;
    elseif strcmp(S(1).subs, 'mag')
      val      = q.mag();
    elseif strcmp(S(1).subs, 'norm')
      val      = q.not();
    elseif strcmp(S(1).subs, 'conj')
      val      = q.ctranspose();
    elseif strcmp(S(1).subs, 'mult')
      val      = q.mult(S(2).subs{:});
    elseif strcmp(S(1).subs, 'multConj')
      val      = q.multConj(S(2).subs{:});
    elseif strcmp(S(1).subs, 'conjMult')
      val      = q.conjMult(S(2).subs{:});

    % components
    elseif strcmp(S(1).subs, 'frwd')
      val      = q.toFrwd();
    elseif strcmp(S(1).subs, 'up')
      val      = q.toUp();
    elseif strcmp(S(1).subs, 'rght')
      val      = q.toRght();
    elseif strcmp(S(1).subs, 'distRad')
      val      = q.toDist();
    elseif strcmp(S(1).subs, 'distDeg')
      val      = 180 * q.toDist() / pi;
    elseif strcmp(S(1).subs, 'rollRad')
      val      = q.toRoll();
    elseif strcmp(S(1).subs, 'rollDeg')
      val      = 180 * q.toRoll() / pi;
    elseif strcmp(S(1).subs, 'rate')
      if length(S) > 1
        val    = q.toRate(S(2).subs{:});
      else
        val    = q.toRate();
      end
    
    % calculating "shift" quaternions
    elseif strcmp(S(1).subs, 'diffQuat')
      val      = q.diffQuatFrwd(S(2).subs{:});    
    elseif strcmp(S(1).subs, 'diffVectFrwd')
      val      = q.diffVectFrwd(S(2).subs{:});
    elseif strcmp(S(1).subs, 'diffVectUp')
      val      = q.diffVectUp(S(2).subs{:});
  
    % quaternion modifiers
    elseif strcmp(S(1).subs, 'scale')
      val      = q.scale(S(2).subs{:});
    elseif strcmp(S(1).subs, 'addRate')
      val      = q.addRate(S(2).subs{:});
    elseif strcmp(S(1).subs, 'forceFrwd')
      val      = q.forceFrwd(S(2).subs{:});
    elseif strcmp(S(1).subs, 'forceUp')
      val      = q.forceUp(S(2).subs{:}); 
    elseif strcmp(S(1).subs, 'forceRollRad')
      val      = q.forceRoll(S(2).subs{:});
    elseif strcmp(S(1).subs, 'forceRollDeg')
      val      = q.forceRoll(pi * S(2).subs{:} / 180);
 
    % vector operations
    elseif strcmp(S(1).subs, 'rotateVectFrwd')
      val      = q.rotateVectFrwd(S(2).subs{:});
    elseif strcmp(S(1).subs, 'rotateVectRvrs')
      val      = q.rotateVectRvrs(S(2).subs{:});
    
    % rotation state conversions 
    elseif strcmp(S(1).subs, 'eulerRad')
      val      = q.toEuler();
    elseif strcmp(S(1).subs, 'eulerDeg')
      val      = 180 * q.toEuler() / pi;
    elseif strcmp(S(1).subs, 'axisAngle')
      val      = q.toAxisAngle();
    elseif strcmp(S(1).subs, 'matrix')
      val      = q.toMatrix();
    end

  % direct access to "val" property
  elseif   strcmp(S(1).type, '()')
    if    (length(S(1).subs) < 1)
      val      = q.val;
    elseif strcmp(S(1).subs{1},':')
      val      = q.val;
    else
      val      = zeros(size(S(1).subs{1}));
      for i=1:length(S(1).subs{1})
        val(i) = q.val(S(1).subs{1}(i));
      end
    end

  % unable to process operation
  else
    error("unsupported function");
  end
end


%% basic operations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% quaternion magnitude
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function val = mag(q)
  val        = norm(q.val);
end


%% quaternion normalize
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = not(q)  % norm(q)
  mag      = norm(q.val);
  if mag > q.epsilon
    q.val  = q.val ./ mag;
  end
end


%% quaternion conjugate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = ctranspose(q)  % conj(q)
  q.val    = [q.val(1), -q.val(2), -q.val(3), -q.val(4)];
end


%% quaternion multiply
%  q = q1 * q2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = mult(q1, q2)
  q1       = q1.val;
  q2       = q2.val;
  q(1)     = q2(1)*q1(1) - q2(2)*q1(2) - q2(3)*q1(3) - q2(4)*q1(4);
  q(2)     = q2(1)*q1(2) + q2(2)*q1(1) - q2(3)*q1(4) + q2(4)*q1(3);
  q(3)     = q2(1)*q1(3) + q2(2)*q1(4) + q2(3)*q1(1) - q2(4)*q1(2);
  q(4)     = q2(1)*q1(4) - q2(2)*q1(3) + q2(3)*q1(2) + q2(4)*q1(1);
  q        = quat(q);
end


%% quaternion multiply
%  q = q1 * q2'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = multConj(q1, q2)
  q1       = q1.val;
  q2       = q2.val;
  q(1)     = q2(1)*q1(1) + q2(2)*q1(2) + q2(3)*q1(3) + q2(4)*q1(4);
  q(2)     = q2(1)*q1(2) - q2(2)*q1(1) + q2(3)*q1(4) - q2(4)*q1(3);
  q(3)     = q2(1)*q1(3) - q2(2)*q1(4) - q2(3)*q1(1) + q2(4)*q1(2);
  q(4)     = q2(1)*q1(4) + q2(2)*q1(3) - q2(3)*q1(2) - q2(4)*q1(1);
  q        = quat(q);
end


%% quaternion multiply
%  q = q1' * q2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = conjMult(q1, q2)
  q1       = q1.val;
  q2       = q2.val;
  q(1)     = q2(1)*q1(1) + q2(2)*q1(2) + q2(3)*q1(3) + q2(4)*q1(4);
  q(2)     = q2(2)*q1(1) - q2(1)*q1(2) + q2(3)*q1(4) - q2(4)*q1(3);
  q(3)     = q2(3)*q1(1) - q2(1)*q1(3) - q2(2)*q1(4) + q2(4)*q1(2);
  q(4)     = q2(4)*q1(1) - q2(1)*q1(4) + q2(2)*q1(3) - q2(3)*q1(2);
  q        = quat(q);
end


%% create via single vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% quaternion from forward vector (tries to constrain up component)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromFrwdSafe(q, f)
  f        = f ./ norm(f);
  if abs(f(1)) < q.epsilon && abs(f(2)) < q.epsilon
    q      = quat(0.7071, 0, -sign(f(3))*0.7071, 0);
    return
  else
    r      = [-f(2), f(1), 0];
  end
  r        = r ./ norm(r);
  u        = cross(f, r);
  M        = [f(1), r(1), u(1);  ...
              f(2), r(2), u(2);  ...
              f(3), r(3), u(3)];
  q        = q.fromMatrix(M);
end


%% quaternion from forward vector (optimized for number of operations)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromFrwdFast(q, f)
  norm     = sqrt(f(1)*f(1) + f(2)*f(2) + f(3)*f(3));
  real     = norm + f(1);
  if (real > q.epsilon * norm)
    q.val  = [real, 0, -f(3), f(2)];
    q.val  = q.val / sqrt(sum(q.val.^2)); 
  else
    q.val  = [0, 0, 0, 1];
  end
end


%% quaternion from up vector (constrains forward component)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromUpSafe(q, u)
  u        = u ./ norm(u);
  if abs(u(1)) < q.epsilon && abs(u(3)) < q.epsilon
    q      = quat(0.7071, -sign(u(2))*0.7071, 0, 0);
    return
  elseif u(3) >= 0
    f      = [u(3), 0, -u(1)];
  else
    f      = [-u(3), 0, u(1)];
  end
  f        = f ./ norm(f);
  r        = cross(u, f);
  M        = [f(1), r(1), u(1);  ...
              f(2), r(2), u(2);  ...
              f(3), r(3), u(3)];
  q        = q.fromMatrix(M);
end


%% quaternion from up vector (optimized for number of operations)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromUpFast(q, u)
  norm     = sqrt(u(1)*u(1) + u(2)*u(2) + u(3)*u(3));
  real     = norm + u(3);
  if (real > q.epsilon * norm)
    q.val  = [real, -u(2), +u(1), 0];
    q.val  = q.val / sqrt(sum(q.val.^2)); 
  else
    q.val  = [0, 1, 0, 0];
  end
end


%% create via two vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% quaternion from forward up (up vector get projected onto forward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromFrwdUp(q, f_org, u_org, mode)
  % normalize reference (forward) vector
  m        = norm(f_org);
  f        = f_org(:)./m;

  % check frwd magnitude
  if m < q.epsilon
    q      = quat;
    return
  end

  % ortho normalize up vector
  [u, m]   = orthonorm(u_org, f);

  % check forward magnitude
  if m < q.epsilon
    if     nargin < 4 || strcmp(mode, "up")
      q    = quat("up",   u_org);
    elseif strcmp(mode, "frwd")
      q    = quat("frwd", f_org);
    else
      error("problem with mode argument");
    end
    return
  end
    
  % calcuate the quaternion
  r        = cross(u, f);
  M        = [f(1), r(1), u(1);  ...
              f(2), r(2), u(2);  ...
              f(3), r(3), u(3)];
  q        = q.fromMatrix(M);
end


%% quaternion from forward up (up vector get projected onto forward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromFrwdUpRght(q, f, u, r)
  % normalize reference (forward) vector
  m        = norm(f);
  f        = f(:)./m;

  % check frwd magnitude
  if m < q.epsilon
    q      = quat;
    return
  end

  % ortho normalize up vector
  [u, m]   = orthonorm(u, f);

  % calulate right vector 
  if m >= q.magThresh
    r      = cross(u, f);
  else
    [r, m] = orthonorm(r, f);
    if m >= q.magThresh
      u    = cross(f, r);
    else
      q    = quat("frwd", f);
      return
    end
  end
  
  % calcuate the quaternion
  M        = [f(1), r(1), u(1);  ...
              f(2), r(2), u(2);  ...
              f(3), r(3), u(3)];
  q        = q.fromMatrix(M);
end


%% quaternion from up forward (forward vector get projected onto up)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromUpFrwd(q, u_org, f_org, mode)
  % normalize reference (up) vector
  m        = norm(u_org);
  u        = u_org(:)./m;

  % check up magnitude
  if m < q.epsilon
    q      = quat;
    return
  end

  % ortho normalize frwd vector
  [f, m]   = orthonorm(f_org, u);
  
  % check forward magnitude
  if m < q.epsilon
    if     nargin < 4 || strcmp(mode, "up")
      q    = quat("up",   u);
    elseif strcmp(mode, "frwd")
      q    = quat("frwd", f_org);
    else
      error("invalid mode");
    end
    return
  end
  
  % calcuate the quaternion
  r        = cross(u, f);     
  M        = [f(1), r(1), u(1);  ...
              f(2), r(2), u(2);  ...
              f(3), r(3), u(3)];
  q        = q.fromMatrix(M);
end


%% quaternion from up forward (forward vector get projected onto up)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromUpFrwdRght(q, u, f, r)
  % normalize reference (up) vector
  m        = norm(u);
  u        = u(:)./m;

  % check up magnitude
  if m < q.epsilon
    q      = quat;
    return
  end

  % ortho normalize frwd vector
  [f, m]   = orthonorm(f, u);
  
  % calculuate right vector
  if  m >= q.magThresh
    r      = cross(u, f);
  else
      
    [r, m] = orthonorm(r, u);
    if m >= q.magThresh
      f    = cross(r, u);
    else
      q    = quat("up", u);
      return
    end
  end

  % calcuate the quaternion
  M        = [f(1), r(1), u(1);  ...
              f(2), r(2), u(2);  ...
              f(3), r(3), u(3)];
  q        = q.fromMatrix(M);
end


%% quaternion rotation between two vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromVectors(q, u, v)
  norm     = sqrt ((u(1)*u(1) + u(2)*u(2) + u(3)*u(3)) *   ...
                   (v(1)*v(1) + v(2)*v(2) + v(3)*v(3)));
  real     = norm + u(1)*v(1) + u(2)*v(2) + u(3)*v(3);
  if (real > q.epsilon * norm)
    q.val  = [real                 ,   ...
              u(2)*v(3) - u(3)*v(2),   ...
              u(3)*v(1) - u(1)*v(3),   ...
              u(1)*v(2) - u(2)*v(1)];
  elseif abs(u(1)) > abs(u(3))
    q.val  = [real, -u(2), u(1), 0];
  else
    q.val  = [real, 0, -u(3), u(2)];
  end
  q        = ~q;
end


%% components
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% quaternion forward component
%  v = q / [1, 0, 0]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = toFrwd(q)
  q        = q.val;
  v        = [2.0 * (0.5 - q(3)*q(3) - q(4)*q(4)),       ...
              2.0 * (q(2)*q(3) + q(1)*q(4)),             ...
              2.0 * (q(2)*q(4) - q(1)*q(3))];
end


%% quaternion up component
%  v = q / [0, 0, 1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = toUp(q)
  q        = q.val;
  v        = [2.0 * (q(2)*q(4) + q(1)*q(3)),             ...
              2.0 * (q(3)*q(4) - q(1)*q(2)),             ...
              2.0 * (0.5 - q(2)*q(2) - q(3)*q(3))];
end


%% quaternion right component
%  v = q / [0, 1, 0]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = toRght(q)
  q        = q.val;
  v        = [2.0 * (q(2)*q(3) - q(1)*q(4)),             ...
              2.0 * (0.5 - q(2)*q(2) - q(4)*q(4)),       ...
              2.0 * (q(3)*q(4) + q(1)*q(2))];
end


%% quaternion to angle rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = toDist(q)
  out        = 2 * acos(q.val(1));
end


%% quaternion roll component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = toRoll(q)
  sinr_cosp  = 2.0 * (q(1)*q(2) + q(3)*q(4));
  cosr_cosp  = 1.0 - 2.0 * (q(2)*q(2) + q(3)*q(3));
  out        = -atan2(sinr_cosp, cosr_cosp);
end


%% quaternion to angle rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = toRate(q, weight)
  temp       = q.toAxisAngle();
  if nargin > 1
    temp(1)  = temp(1) * weight;
  end
  out        = temp(1) * temp(2:4);
end


%% calcuating "shift" quaternions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% quaternion difference
%  q = q1' * q2
%  if q(1) < 0
%    q = -q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = diffQuatFrwd(q1, q2)
  q        = q1.conjMult(q2);
  if (q.val(1) < 0)
    q      = -q;
  end  
end


%% quaternion difference
%  q = q2' * q1
%  if q(1) < 0
%    q = -q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = diffQuatRvrs(q1, q2)
  q        = q2.conjMult(q1);
  if (q.val(1) < 0)
    q      = -q;
  end  
end


%% quaternion difference between forward vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = diffVectFrwd(q_in, frwd, mode)
  q        = q_in / q_in.forceFrwd(frwd,  mode);
end


%% quaternion difference between up vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = diffVectUp(q_in, up, mode)
  q        = q_in / q_in.forceUp(up, mode);
end


%% quaternion modifiers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% quaternion scale 
%  1 - converts to axis-angle value 
%  2 - applies scale to magnitude
%  3 - converts back to quaternion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = scale(q, scalar)
  ang      = q.toAxisAngle();
  ang(1)   = scalar * ang(1);
  q        = quat("axisAngle", ang);  
end


%% quaternion add rate
%  q = q + scalar * 0.5 * [0, g] * q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = addRate(q, g, scalar)
   q1      = q.val;
   dq      = [-q1(2)*g(1) - q1(3)*g(2) - q1(4)*g(3),     ...
               q1(1)*g(1) + q1(3)*g(3) - q1(4)*g(2),     ...
               q1(1)*g(2) - q1(2)*g(3) + q1(4)*g(1),     ...
               q1(1)*g(3) + q1(2)*g(2) - q1(3)*g(1)];
   if nargin > 2
     dq    = dq * scalar;
   end
   q       = q + 0.5 * dq;
end


%% quaternion difference between up vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = forceFrwd(q_in, frwd, mode)
  if nargin < 2 || strcmp(mode, "frwd")
    q      = quat("frwdUpRght", frwd, q_in.toUp(), q_in.toRght());
  elseif strcmp(mode, "up")
    q      = quat("upFrwdRght", q_in.toUp(), frwd, q_in.toRght());
  else
    error("invalid mode");
  end
end


%% quaternion difference between up vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = forceUp(q_in, up, mode)
  if nargin < 2 || strcmp(mode, "up")
    q      = quat("upFrwdRght", up, q_in.toFrwd(), q_in.toRght());
  elseif strcmp(mode, "frwd")
    q      = quat("frwdUpRght", q_in.toFrwd(), up, q_in.toRght());
  else
    error("invalid mode");
  end
end


%% quaternion difference between up vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q  = forceRoll(q, roll)
  % extract core vector
  q         = q.val;
    
  % extract pitch (y-axis rotation)
  sinp      = 2.0 * (q(1)*q(3) - q(4)*q(2));
  if (abs(sinp) >= 1)
    ang2    = -sign(sinp) * pi / 2;
  else
    ang2    = -asin(sinp);
  end

  % extract yaw (z-axis rotation)
  siny_cosp = 2.0 * (q(1)*q(4) + q(2)*q(3));
  cosy_cosp = 1.0 - 2.0 * (q(3)*q(3) + q(4)*q(4));
  ang1      = atan2(siny_cosp, cosy_cosp);

  % create quaternion
  q         = quat("eulerRad", ang1, ang2, roll);
end


%% vector operations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% vector rotate forward
%  v = q * [0, v] * q'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = rotateVectFrwd(q, v)
  q          = q.val;
  out(1)     = 2.0 * (v(1) * (0.5 - q(3)*q(3) - q(4)*q(4))    ...
                    + v(2) * (q(2)*q(3) - q(1)*q(4))          ...
                    + v(3) * (q(2)*q(4) + q(1)*q(3)));
  out(2)     = 2.0 * (v(1) * (q(2)*q(3) + q(1)*q(4))          ...
                    + v(2) * (0.5 - q(2)*q(2) - q(4)*q(4))    ...
                    + v(3) * (q(3)*q(4) - q(1)*q(2)));
  out(3)     = 2.0 * (v(1) * (q(2)*q(4) - q(1)*q(3))          ...
                    + v(2) * (q(3)*q(4) + q(1)*q(2))          ...
                    + v(3) * (0.5 - q(2)*q(2) - q(3)*q(3)));
end


%% vector rotate reverse
%  v = q' * [0, v] * q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = rotateVectRvrs(q, v)
  q          = q.val;
  out(1)     = 2.0 * (v(1) * (0.5 - q(3)*q(3) - q(4)*q(4))    ...
                    + v(2) * (q(2)*q(3) + q(1)*q(4))          ...
                    + v(3) * (q(2)*q(4) - q(1)*q(3)));
  out(2)     = 2.0 * (v(1) * (q(2)*q(3) - q(1)*q(4))          ...
                    + v(2) * (0.5 - q(2)*q(2) - q(4)*q(4))    ...
                    + v(3) * (q(3)*q(4) + q(1)*q(2)));
  out(3)     = 2.0 * (v(1) * (q(2)*q(4) + q(1)*q(3))          ...
                      + v(2) * (q(3)*q(4) - q(1)*q(2))          ...
                  + v(3) * (0.5 - q(2)*q(2) - q(3)*q(3)));
end


%% quaternion conversions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% from axis angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromAxisAngle(q, axisAngle)
  angle    = sin(axisAngle(1)/2);
  q.val(1) = cos(axisAngle(1)/2);
  q.val(2) = angle * axisAngle(2);
  q.val(3) = angle * axisAngle(3);
  q.val(4) = angle * axisAngle(4);
end


%% to axis angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = toAxisAngle(q)
  if q.val(1) > 1.0 - q.epsilon
    out    = [0.0, 1.0, 0.0, 0.0];
  else
    scale  = 1 / sqrt(1 - q.val(1) * q.val(1));
    out(1) = 2 * acos(q.val(1));
    out(2) = scale * q.val(2);
    out(3) = scale * q.val(3);
    out(4) = scale * q.val(4);
  end
end


%% from Euler angles (radians)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromEuler(q, ang)
  cy = cos(ang(1) * 0.5);
  sy = sin(ang(1) * 0.5);
  cp = cos(-ang(2) * 0.5);
  sp = sin(-ang(2) * 0.5);
  cr = cos(-ang(3) * 0.5);
  sr = sin(-ang(3) * 0.5);

  % conversion to quaternion
  q.val(1) = cy*cr*cp + sy*sr*sp;
  q.val(2) = cy*sr*cp - sy*cr*sp;
  q.val(3) = cy*cr*sp + sy*sr*cp;
  q.val(4) = sy*cr*cp - cy*sr*sp;
end


%% to Euler angles (radians)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ang = toEuler(q)
  % extract core vector
  q         = q.val;
    
  % roll (x-axis rotation)
  sinr_cosp = 2.0 * (q(1)*q(2) + q(3)*q(4));
  cosr_cosp = 1.0 - 2.0 * (q(2)*q(2) + q(3)*q(3));
  ang(3)    = -atan2(sinr_cosp, cosr_cosp);

  % pitch (y-axis rotation)
  sinp      = 2.0 * (q(1)*q(3) - q(4)*q(2));
  if (abs(sinp) >= 1)
    ang(2)  = -sign(sinp) * pi / 2;
  else
    ang(2)  = -asin(sinp);
  end

  % yaw (z-axis rotation)
  siny_cosp = 2.0 * (q(1)*q(4) + q(2)*q(3));
  cosy_cosp = 1.0 - 2.0 * (q(3)*q(3) + q(4)*q(4));
  ang(1)    = atan2(siny_cosp, cosy_cosp);
end


%% from rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromMatrix(q, M)
  % calculate matrix trace
  tr       = M(1,1) + M(2,2) + M(3,3);

  % calculate the quaterion
  if (tr > 0) 
    S      = sqrt(tr+1.0) * 2; 
    v(1)   = 0.25 * S;
    v(2)   = (M(3,2) - M(2,3)) / S;
    v(3)   = (M(1,3) - M(3,1)) / S;
    v(4)   = (M(2,1) - M(1,2)) / S;
  elseif ((M(1,1) > M(2,2)) && (M(1,1) > M(3,3))) 
    S      = sqrt(1.0 + M(1,1) - M(2,2) - M(3,3)) * 2; 
    v(1)   = (M(3,2) - M(2,3)) / S;
    v(2)   = 0.25 * S;
    v(3)   = (M(1,2) + M(2,1)) / S; 
    v(4)   = (M(1,3) + M(3,1)) / S; 
  elseif (M(2,2) > M(3,3))  
    S      = sqrt(1.0 + M(2,2) - M(1,1) - M(3,3)) * 2;
    v(1)   = (M(1,3) - M(3,1)) / S;
    v(2)   = (M(1,2) + M(2,1)) / S;
    v(3)   = 0.25 * S;
    v(4)   = (M(2,3) + M(3,2)) / S;
  else 
    S      = sqrt(1.0 + M(3,3) - M(1,1) - M(2,2)) * 2;
    v(1)   = (M(2,1) - M(1,2)) / S;
    v(2)   = (M(1,3) + M(3,1)) / S;
    v(3)   = (M(2,3) + M(3,2)) / S;
    v(4)   = 0.25 * S;
  end

  % create the quat objection
  q.val  = v;
end


%% to rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function M = toMatrix(q)
  % extract components
  w = q.val(1);
  x = q.val(2);
  y = q.val(3);
  z = q.val(4);

  % calculate the matrix
  M = [1-2*y*y-2*z*z,    2*x*y-2*w*z,    2*x*z+2*w*y;
         2*x*y+2*w*z,  1-2*x*x-2*z*z,    2*y*z-2*w*x;
         2*x*z-2*w*y,    2*y*z+2*w*x,  1-2*x*x-2*y*y];
end


%% overload operations (Matlab only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% urinary minus
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = uminus(q)
  q.val    = [-q.val(1), -q.val(2), -q.val(3), -q.val(4)];
end


%% quaternion addition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = plus(q1, q2)
  % extract values
  if isobject(q1)
    q1     = q1.val;
  end
  if isobject(q2)
    q2     = q2.val;
  end
  
  % perform operation
  q        = quat(q1 + q2);
end


%% quaternion subtraction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = minus(q1, q2)
  % extract values
  if isobject(q1)
    q1     = q1.val;
  end
  if isobject(q2)
    q2     = q2.val;
  end
  
  % perform operation
  q        = quat(q1 - q2);
end


%% quaternion element-wise multiplication
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = times(q1, q2)
  % extract values
  if isobject(q1)
    q1     = q1.val;
  end
  if isobject(q2)
    q2     = q2.val;
  end
  
  % perform operation
  q        = quat(q1 .* q2);
end


%% quaternion element-wise division
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = rdivide(q1, q2)
  % extract values
  if isobject(q1)
    q1     = q1.val;
  end
  if isobject(q2)
    q2     = q2.val;
  end
  
  % perform operation
  q        = quat(q1 ./ q2);
end


%% quaternion multiply
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = mtimes(q1, q2)
  % use axis-angle scale provided scalar input
  if     ~isobject(q1) && (length(q1) == 1)
    q      = q2.scale(q1);
  elseif ~isobject(q2) && (length(q2) == 1)
    q      = q1.scale(q2);

  % multiply two quaternions (no scalars vals)
  else
    if ~isobject(q1)
      q1   = quat(q1);
    end
    if ~isobject(q2)
      q2   = quat(q2);
    end
    q      = q1.mult(q2);
  end
end


%% quaternion divide (forward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = mrdivide(q, arg)
  % check for quaternions object
  if     isobject(arg)
    out      = q.diffQuatFrwd(arg);
    
  % check for quaternion array
  elseif (length(arg) == 4)
    out      = q.diffQuatFrwd(quat(arg));
    
  % check for vector (rotate forward)
  elseif (length(arg) == 3)
    out      = q.rotateFrwd(arg);
    
  % check for scalar
  elseif (length(arg) == 1)
    out      = q.scale(1/arg);
  end
end


%% quaternion divide (reverse)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = mldivide(q, arg)
  % check for quaternions object
  if     isobject(arg)
    out      = q.diffQuatRvrs(arg);
    
  % check for quaternion array
  elseif (length(arg) == 4)
    out      = q.diffQuatRvrs(quat(arg));

  % check for vector (rotate reverse)
  elseif (length(arg) == 3)
    out      = q.rotateRvrs(arg);
    
  % check for scalar
  elseif (length(arg) == 1)
    out      = q.scale(1/arg);
  end
end


%% end of class definition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end % methods

end % classdef


%% helper function(s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% orthonormalize vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [v, mag] = orthonorm(v, ref)
  v        = v(:);
  ref      = ref(:);
  D        = dot(v, ref);
  v        = v - D * ref;
  mag      = norm(v);
  v        = v./mag;
end