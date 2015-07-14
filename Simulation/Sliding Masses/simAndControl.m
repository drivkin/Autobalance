clear all
clc
starting = 1


%compute symbolic linearization matrix
%EKFsymbolicDiscrete;
%BetterEKFSymbolic;

t0 = 0;
tf = 30;

%global constants, these can be used by simulator, estimator, and
%controller
global T M m I comB mpInit;

T = .01;
%mass of the body
M = 10;
%masses of the sliding masses
m = [.5;.5;.5];
%Moment of inertia of the body without the masses. Doesn't have to be
%diagonal
I = diag([1 2 3]);
%center of mass of the body
comB = [.01;.01;-.2];



N = ceil((tf-t0)/T); %number of points

% the states are
% orientation quaterion (1:4)
% angular velocity (5:7)
% position of masses(8:16)
% velocity of masses(17:19)
% acceleration of masses(20:22)



w0 = [.1;0;0]; %initial body angular veloctiy wrt inertial frame

%initial mass positions
rm10 = [1;.05;-.05];
rm20 = [0.05;1;-.05];
rm30 = [0;0;-1];

mpInit = [rm10;rm20;rm30];

vm0 = [0;0;0]; %initial linear  velocity of masses
am0 = [0;0;0]; %initial linear acceleration of masses

%q0 = angle2quat(0,0,.1); %initial orientation
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

%xhat is the time history of estimates, these map differently from the
%states for the simulation, so don't forget to remap
xhat = zeros(30,N);

gInert = [0 0 -9.8]';
lastReset = 0;
for i=2:N
    %simulation
    tcurr
    [tsim rsim] = ode45(@ABSMdyn,[tcurr tcurr+T],r(:,i-1));
    rsim = rsim';
    rsim = rsim(:,end);
    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %USE THIS CODE FOR EKF ESTIMATE FEEDBACK BALANCING
%     
%     
%     
%     %reset every 10 seconds, including filter reset
%     if(tcurr - lastReset > 100)
%         qrst = q0;
%         wrst = w0;
%         rst = rsim;
%         rst(1:4) = qrst';
%         rst(5:7) = wrst;
%         rst(8:16) = rc(8:16);
%         r(:,i) = rst;
%         rsim = rst;
%            %estimation
%         %get y, which is w(1:3) and gbody(4:6)
%         q = rsim(1:4);
%         gBody = quat2dcm(q')*gInert;
%         vm1 = [rsim(17) 0 0]';
%         vm2 = [0 rsim(18) 0]';
%         vm3 = [0 0 rsim(19)]';
%         y = [rsim(5:7);vm1;vm2;vm3; gBody];
%         
%         %update estimate
%         xhat(:,i) = ABSMEKFupdate(y,1);
%         lastReset = tcurr;
%     else
%         %estimation
%         %get y, which is w(1:3) and gbody(4:6)
%         q = rsim(1:4);
%         gBody = quat2dcm(q')*gInert;
%         vm1 = [rsim(17) 0 0]';
%         vm2 = [0 rsim(18) 0]';
%         vm3 = [0 0 rsim(19)]';
%         y = [rsim(5:7);vm1;vm2;vm3; gBody];
%         
%         %update estimate
%         xhat(:,i) = ABSMEKFupdate(y,0);
%     end
%     
%     %control
%     rc = controlUpdate2(tcurr,rsim,xhat(:,i));
%     %store simulated state
%     r(:,i) = rc;
%     
%     
%     
%     %reset if it tilts too much
%     %     [yaw pitch roll] = quat2angle(rc(1:4)');
%     %
%     %         if(abs(pitch) > pi/4 || abs(roll)>pi/4)
%     %             qrst = q0;
%     %             wrst = w0;
%     %             rst = zeros(22,1);
%     %             rst(1:4) = qrst';
%     %             rst(5:7) = wrst;
%     %             rst(8:16) = rc(8:16);
%     %             r(:,i) = rst;
%     %         end
%     %     if(norm(rc(5:7))>.3)
%     %         %qrst = angle2quat(0,0,0);
%     %         qrst = q0;
%     %         wrst = w0;
%     %         rst = zeros(22,1);
%     %         rst(1:4) = qrst';
%     %         rst(5:7) = wrst;
%     %         rst(8:16) = rc(8:16);
%     %         r(:,i) = rst;
%     %     end
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
   % USE THIS CODE FOR EULER ANGLE FEEDBACK BALANCING
       %compute control
       rc = controlUpdate1(rsim);
       %store state vector with control added
       r(:,i) = rc;
    
       %if the oscillations become too intense, reset
       if(norm(rc(5:7))>.3)
           %qrst = angle2quat(0,0,.1);
           qrst = computeEquilibriumPosition(computeTotalCoM(M,m,comB,rc(8:16)));
           wrst = [.1;0;0];
           rst = zeros(22,1);
           rst(1:4) = qrst';
           rst(5:7) = wrst;
           rst(8:16) = rc(8:16);
           r(:,i) = rst;
       end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %update time and time vector
    tcurr = tcurr+T;
    t = [t tcurr];
end

%%
close all;


q = r(1:4,:);
w = r(5:7,:);

figure
plotVector(t,q,'quat');

figure
plotVector(t,w,'omega');

%compute overall CoM
for i = 1:N
    com(:,i) = computeTotalCoM(M,m,comB,r(8:16,i));
end

for i = 1:N
    comHat(:,i) = computeTotalCoM(M,m,xhat(28:30,i),xhat(4:12,i));
end

figure
plotVector(t,com,'CM');
% CMest = comHat;
% plotVector(t,CMest,'CMest');
% legend('CM_1','CM_2','CM_3','CMEST_1','CMEST_2','CMEST_3');
% 
% %%
% %plotting estimated values
% 
% 
% % xhat (Estimator state) is
% %wBody (1:3)
% %position of masses(4:12)
% %velocity of masses(13:21)
% %gBody (22:24) gravity vector in body frame
% %Ibody (25:27) MOI of the body without the masses (diagonal for now)
% %CoM Body (28:30) Center of mass of the body without the masses
% %estimated MOI matrix
% MOI = xhat(25:27,:);
% figure
% plotVector(t,MOI,'BMOIest');
% 
% 
% 
