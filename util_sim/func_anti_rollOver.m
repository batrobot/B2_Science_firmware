% func_anti_rollOver: prevent roll-over in as5048B (it rolles over at 360 deg)
% angle: encoder measurments, in deg, measurements are not filters, 4-by-1
% vecot
%   angle(1): right forelimb
%   angle(2): left forelimb
%   angle(3): right leg
%   angle(4): left leg
% angle_prev: measurments from previous sample time
%   angle_prev(1): right forelimb
%   angle_prev(2): left forelimb
%   angle_prev(3): right leg
%   angle_prev(4): left leg
% angle_aro: anti-rolled over angles
% vecot
%   angle_aro(1): right forelimb
%   angle_aro(2): left forelimb
%   angle_aro(3): right leg
%   angle_aro(4): left leg
% by ALireza Ramezani, 8-31-2015, Champaign, IL
function angle_aro = func_anti_rollOver(angle,angle_prev)
%#codegen
angle_difference = zeros(4,1);
angle_aro = zeros(4,1);

% guess max angular changes during one sample time
global MAX_ANGLE_DIFFERENCE;
global ANTI_ROLLOVER_CORRECTION;
global ROLLOVER_FLAG;
global MAX_RP_ANGLE_RIGHT;
global MAX_DV_ANGLE_RIGHT;
global MIN_RP_ANGLE_RIGHT;
global MIN_DV_ANGLE_RIGHT;
global MAX_RP_ANGLE_LEFT;
global MAX_DV_ANGLE_LEFT;
global MIN_RP_ANGLE_LEFT;
global MIN_DV_ANGLE_LEFT;

DEG2RAD = pi/180; % rad\deg

% rollover matrix
ANTI_ROLLOVER_CORRECTION_MATRIX = ANTI_ROLLOVER_CORRECTION*eye(4,4);

% initiate angle aro
angle_aro = angle;

% ANTI_ROLLOVER_THRESHOLD = ANTI_ROLLOVER_CORRECTION - MAX_ANGLE_DIFFERENCE; % deg

% comptue the difference
angle_difference = angle - angle_prev;


% for i=1:4
%     if (angle_difference(i) > -ANTI_ROLLOVER_THRESHOLD)||(angle_difference(i) < ANTI_ROLLOVER_THRESHOLD)
%         if(angle_difference(i)<0)
%             angle_aro(i) = angle(i) + ANTI_ROLLOVER_CORRECTION;
%         else
%             angle_aro(i) = angle(i) - ANTI_ROLLOVER_CORRECTION;
%         end
%     end
% end


for i=1:4
    if (angle_difference(i) > MAX_ANGLE_DIFFERENCE)||(angle_difference(i) < -MAX_ANGLE_DIFFERENCE)
        if(angle_difference(i)<0)
            ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) + 1;
        else
            ROLLOVER_FLAG(i) = ROLLOVER_FLAG(i) - 1;
        end
    end
end

angle_aro = angle + ANTI_ROLLOVER_CORRECTION_MATRIX*ROLLOVER_FLAG;


% convert angles to radian
angle_aro = DEG2RAD*angle_aro;

% calibration materials
max_angle = DEG2RAD*[MAX_RP_ANGLE_RIGHT,MAX_RP_ANGLE_LEFT,MAX_DV_ANGLE_RIGHT,MAX_DV_ANGLE_LEFT].';
min_angle = DEG2RAD*[MIN_RP_ANGLE_RIGHT,MIN_RP_ANGLE_LEFT,MIN_DV_ANGLE_RIGHT,MIN_DV_ANGLE_LEFT].';
delta_angle_matrix = diag(max_angle-min_angle);

% calibrate angles
angle_aro = angle_aro-min_angle;

end