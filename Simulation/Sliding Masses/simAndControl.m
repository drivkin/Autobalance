clear all
clc
starting = 1
t0 = 0;
tf = 30;

%global constants, these can be used by simulator, estimator, and
%controller
global T M m I comB;
T = .01;
%mass of the body
M = 10;
%masses of the sliding masses
m = [.5;.5;.5];
%Moment of inertia of the body without the masses. Doesn't have to be
%diagonal
I = diag([1 2 3]);
%center of mass of the body
comB = [-.01;-.01;-.2];



N = ceil((tf-t0)/T); %number of points

% the states are 
% orientation quaterion (1:4)
% angular velocity (5:7)
% position of masses(8:16)
% velocity of masses(17:19)
% acceleration of masses(20:22)



w0 = [0.1;.1;.1]; %initial body angular veloctiy wrt inertial frame

%initial mass positions
rm10 = [1;1;.1];
rm20 = [1;1;.1];
rm30 = [0;0;-1];

vm0 = [0;0;0]; %initial linear  velocity of masses
am0 = [0;0;0]; %initial linear acceleration of masses

%q0 = angle2quat(0,0,0); %initial orientation
%if you want the initial orientation to be an equilibrium
q0 = computeEquilibriumPosition(computeTotalCoM(M,m,comB,[rm10;rm20;rm30]));


rinit = [q0';w0;rm10;rm20;rm30;vm0;am0];


%just simulation (no control)
%[tsim rsim] = ode45(@ABSMdyn,[t0 tf],rinit);


%this is for simulation, estimation, and control. Samples at regular
%intervals
tcurr = t0;
t = t0;


%r is a time history of states
r = zeros(22,N);
r(:,1) = rinit;
gInert = [0 0 -9.8]';
for i=2:N
    %simulation
   tcurr
   [tsim rsim] = ode45(@ABSMdyn,[tcurr tcurr+T],r(:,i-1));
   rsim = rsim';
   rsim = rsim(:,end);

   %compute control
   rc = controlUpdate1(rsim);
   %store state vector with control added
   r(:,i) = rc;
   
   %if the oscillations become too intense, reset
   if(norm(rc(5:7))>.3)
       %qrst = angle2quat(0,0,0);
       qrst = computeEquilibriumPosition(computeTotalCoM(M,m,comB,rc(8:16)));
       wrst = [.1;.1;.1];
       rst = zeros(22,1);
       rst(1:4) = qrst';
       rst(5:7) = wrst;
       rst(8:16) = rc(8:16);
       r(:,i) = rst;
   end
   
   
   %update time and time vector
   tcurr = tcurr+T;
   t = [t tcurr];
end
%%
q = r(1:4,:);
w = r(5:7,:);

close all
figure
plotVector(t,q,'quat');

figure
plotVector(t,w,'omega');

%compute overall CoM
for i = 1:N
    com(:,i) = computeTotalCoM(M,m,comB,r(8:16,i));
end

figure
plotVector(t,com,'CM');



