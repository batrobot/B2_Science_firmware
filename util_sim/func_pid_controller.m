% func_pid_controller: pd position controller
% angle: 4-by-1 vec
%   angle(1): right forelimb
%   angle(2): left forelimb
%   angle(3): right leg
%   angle(4): left leg
% des_angle: desired position 4-by-1 vec
%   des_angle(1): right forelimb
%   des_angle(2): left forelimb
%   des_angle(3): right leg
%   des_angle(4): left leg
% prev_err: memory for err vector 4-by-1 vec
%   prev_err(1): right forelimb
%   prev_err(2): left forelimb
%   prev_err(3): right leg
%   prev_err(4): left leg
% gain: control gain vector 2-by-2 vec
%   gain(1,1:2): right forelimb Kp and Kd
%   gain(1,1:2): left forelimb  Kp and Kd
%   gain(2,1:2): right leg Kp and Kd
%   gain(2,1:2): left leg Kp and Kd
% u: control action, 4-by-1 vec
%   u(1): right forelimb
%   u(2): left forelimb
%   u(3): right leg
%   u(4): left leg
% err: to save err vec in memory
% by ALireza Ramezani, 8-31-2015, Champaign, IL
function [u,err] = func_pid_controller(angle,des_angle,prev_err,gain)
%#codegen
u = zeros(4,1);
err = zeros(4,1); % err
derr = zeros(4,1); % derivative of err
serr = zeros(4,1); % integration of err

% global vars
global MAX_RP_ANGLE_RIGHT;
global MAX_DV_ANGLE_RIGHT;
global MIN_RP_ANGLE_RIGHT;
global MIN_DV_ANGLE_RIGHT;
global MAX_RP_ANGLE_LEFT;
global MAX_DV_ANGLE_LEFT;
global MIN_RP_ANGLE_LEFT;
global MIN_DV_ANGLE_LEFT;

Kp = [gain(1,1),0,0,0;...
      0, gain(1,1),0,0;...
      0, 0,gain(2,1),0;...
      0, 0, 0,gain(2,1)];

Kd = [gain(1,2),0,0,0;...
      0, gain(1,2),0,0;...
      0, 0,gain(2,2),0;...
      0, 0, 0,gain(2,2)];

% compute error and derr
err = angle - des_angle;
derr = err-prev_err;

% computer u
u = -Kp*err -Kd*derr;

% turn-off the controller immediately when hit the limits
DEG2RAD = pi/180; % rad\deg
max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].';
min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].';
delta_max_angle = max_angle-min_angle;
for i=1:4
    if((angle(i)>delta_max_angle(i))||(angle(i)<0))
        u(i) = 0;
    end
end

end