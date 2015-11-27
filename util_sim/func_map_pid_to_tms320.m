% func_map_pid_to_servo: map controller input to tms320 commands
% u: control action, 4-by-1 vec
%   u(1): right forelimb
%   u(2): left forelimb
%   u(3): right leg
%   u(4): left leg
% u_drv: drv8835 control command, 4-by-1 vec
%   u_tms(1): right forelimb
%   u_tms(2): left forelimb
%   u_tms(3): right leg
%   u_tms(4): left leg
%
% NOTE: MCB communicates with TMS320 over PWM, there are three states:
%  1) 50(+-2)% duty cycle: stop
%  2) 0(+-2)-to-50(+-2)% reverse
%  3) 50(+-2)-to-100(+-2)% forward
% by ALireza Ramezani, 8-31-2015, Champaign, IL
function u_tms = func_map_pid_to_tms320(u)
%#codegen
PWM_MAX_DC = 98; % max duty cycle
PWM_MIN_DC = 2; % min duty cycle
PWM_ZERO_DC = 50; % zero command duty cycle

u_tms = zeros(4,1);
s = PWM_ZERO_DC*ones(4,1);
% used for saturating control command to tms320
global PID_SATURATION_THRESHOLD;

% saturate u
for i=1:4
    if u(i)> PID_SATURATION_THRESHOLD
        u(i) = PID_SATURATION_THRESHOLD;
    end
    
    if u(i)< -PID_SATURATION_THRESHOLD
        u(i) = -PID_SATURATION_THRESHOLD;
    end
end

u_tms = u + s;

for i=1:4
    if u_tms(i)> PWM_MAX_DC
        u_tms(i) = PWM_MAX_DC;
    end
    
    if u_tms(i)< PWM_MIN_DC
        u_tms(i) = PWM_MIN_DC;
    end
end

end
% end of code