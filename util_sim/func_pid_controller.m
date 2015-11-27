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
% gain: control gain vector 4-by-1 vec
%   gain(1): right-left forelimb Kp
%   gain(2): right-left leg Kp
%   gain(3): right-left forelimb Kd
%   gain(4): right-left leg Kd
%   gain(5): right-left forelimb Ki
%   gain(6): right-left leg Ki
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
global SAMPLING_INTERVAL;
global ERR_INTEGRALE;

Kp = [gain(1),0,0,0;...
      0, gain(1),0,0;...
      0, 0,gain(2),0;...
      0, 0, 0,gain(2)];

Kd = [gain(3),0,0,0;...
      0, gain(3),0,0;...
      0, 0,gain(4),0;...
      0, 0, 0,gain(4)];
  
Ki = [gain(5),0,0,0;...
      0, gain(5),0,0;...
      0, 0,gain(6),0;...
      0, 0, 0,gain(6)];  

% compute error and derr
err = angle - des_angle;
derr = (err-prev_err)/SAMPLING_INTERVAL;
ERR_INTEGRALE = ERR_INTEGRALE + err;

% computer u
u = -Kp*err -Kd*derr -Ki*ERR_INTEGRALE;

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