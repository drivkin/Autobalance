function [ rc ] = controlUpdate2(t,r, xhat)
%This function implements proportional control using an estimate of the CoM
%position.

%r is the state vector
% the states are 
% orientation quaterion (1:4)
% angular velocity (5:7)
% position of masses(8:16)
% velocity of masses(17:19)
% acceleration of masses(20:22)

%xhat is the state estimate
%estimated states are
%diagonal entries of MOI matrix (1:3)
%angular velocity (4:6)
%center of mass location(7:9)
%gravity vector (10:12)

%rc is the state vector with the control states changed 
global M m T;

persistent integral
if(isempty(integral))
    integral = [0;0;0];
end

%for now, the control is the speed of the mass, not the acceleration
%(acceleration is maintained at zero)

cm = computeTotalCoM(M,m,xhat(28:30),xhat(4:12));

integral = integral+T*cm;

kp = 10;
ki = 0;

control = -kp*cm - ki*integral;


 control(1) = 0;
 control(2) = 0;
 control(3) = 0; %lets try just balancing x and y first

% control(1) = sin(5*t);
% control(2) = sin(5*t);
% control(3) = sin(5*t); %lets try just balancing x and y first

rc = r;

rc(17:19) = control;


end

