%generate data with sample time T using ode45 solver
clear all
t0 = 0;
tf = 10;
T = .01;

N = ceil((tf-t0)/T); %number of points
r = zeros(7,N);

q0 = angle2quat(.01,.02,.03); %initial orientation
w0 = [0;0;0]; %initial velocity

r(:,1) = [q0';w0];

tcurr = t0;
t = t0;
for i=2:N
   [tsim rsim] = ode45(@ABDyn,[tcurr tcurr+T],r(:,i-1));
   rsim = rsim';
   r(:,i) = rsim(:,end);
   tcurr = tcurr+T;
   t = [t tcurr];
end

q = r(1:4,:);
w = r(5:7,:);
%%
close all
figure
plotVector(t,q,'quat');

figure
plotVector(t,w,'omega');

%% 
%compute gravity vector
gInert = [0 0 -9.8]';
for i =1:N
    gBody(:,i) = quat2dcm(q(:,i)')*gInert;
end
%%
%add measurement noise
% gBody = gBody + 1*randn(size(gBody));
% w = w+1*randn(size(w));

save('simData','t','gBody','w');


