function [ rc ] = controlUpdate1(r)
%This function implements control which attempts to bring the CoG to the z
%axis below the CoR. It does this based purely on the tilt of the testbed,
%trying to balance it flat.

%r is the control augmented state vector
%rc is updated control augmented state vector, where the control states are changed

% the states are 
% orientation quaterion (1:4)
% angular velocity (5:7)
% position of masses(8:16)
% velocity of masses(17:19)
% acceleration of masses(20:22)

% yaw = rotation about Z
% pitch = rotation about Y (control with x actuator)
% roll = rotation about X (control with y actuator)

%for now, we keep it simple by ignoring linear acceleration, leaving it zero,
%and using the speed as the control. The controller will be bang-bang. We
%start with the most obvious approach, where we control the weights
%independently based on the sign of the pitch and roll angles. We only move
%the x and y weights. The center of gravity already has to be below the
%center of rotation when control starts.

speed = 2;

kd = 0;

wx = r(5);
wy = r(6);


q = r(1:4);
[yaw pitch roll] = quat2angle(q');


xspeed = -speed*pitch - kd*wy;
yspeed = speed*roll + kd*wx;

rc = r;

rc(17) = xspeed;
rc(18) = yspeed;

end

