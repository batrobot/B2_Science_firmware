% func_lowpass_filter: this is a simple moving average filter(n: window size)
% angle: 4-by-1 vec
%   angle(1): right forelimb
%   angle(2): left forelimb
%   angle(3): right leg
%   angle(4): left leg
% angle_prev: from n-previous sample times, a 4-by-n vector
%   angle_prev(1,i): right forelimb @ time(i)
%   angle_prev(2,i): left forelimb  @ time(i)
%   angle_prev(3,i): right leg @ time(i)
%   angle_prev(4,i): left leg @ time(i)
% angle_f: filtered data, 4-by-1 vec
%   angle_f(1): right forelimb
%   angle_f(2): left forelimb
%   angle_f(3): right leg
%   angle_f(4): left leg
% by ALireza Ramezani, 8-31-2015, Champaign, IL
function angle_f = func_lowpass_filter(angle,angle_prev)
%#codegen
angle_f = zeros(4,1);
angle_sum = zeros(4,1);

% find the length of moving ave window
[~,n] = size(angle_prev);

for i=1:n
    angle_sum = angle_sum + angle_prev(:,i);
end

angle_sum = angle_sum + angle;

angle_f = angle_sum/(n+1);

end
% end of code