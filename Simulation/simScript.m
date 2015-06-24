% This script runs the simulation of an air bearing where the center of
% rotation might not be coincident with the center of mass. Other torque
% sources are ignored. Essentially a 3D pendulum. 

%attitude is expressed using quaternions

%initial condition is just a quaternion expressing the initial orientation
%of the satellite body wrt the inertial frame

clear all
qInit = angle2quat(0,pi/10,0);
wInit = [0 0 0];

t0 = 0;
tf = 10;

[t,rSim] = ode45(@ABDyn,[t0 tf],[qInit wInit]);

q = rSim(:,1:4);
w = rSim(:,5:7);

close all
figure
plot(t,w(:,1));
hold all
plot(t,w(:,2));
plot(t,w(:,3));
title('omega');

figure
plot(t,q(:,1));
hold all
plot(t,q(:,2));
plot(t,q(:,3));
plot(t,q(:,4));
title('q');

%%
gb = [0; 0; -9.8];
for i = 1:size(q,1)
    gsim(:,i) = quat2dcm(q(i,:))*gb;
end
wsim = w';
tsim = t;

save('simData','gsim','wsim','tsim');