%design of EKF
%In this design, we assume that the MOI matrix is diagonal. Also, the
%orientation of the body is not modeled, but we do have a gravity vector from an
%accelerometer.

clear all

%generate linearized A matrix
EKFsymbolic;

%load data
load('simData');
gsim = gBody;
wsim = w;
tsim = t;

%state is [I11;I22;I33;w;cg;g] where the first three are the diagonal terms
%of the MOI matrix, w is the angular momentum expressed in the body frame,
%cg is the center of gravity vector (in the body frame) and g is the
%gravity vector( in the body frame)


%the guess for MOI should be based on some simple model

%the initial angular velocity is zero (because we start the oscillations
%tilted but stationary)

%guess for cg is 0;0;0 because we guess that it's close to balanced

%guess for g is [0; 0; -9.8]. This is reasonable because it should be
%pretty close to horizontal
xinit = [.11;
        .1;
        .1;
       wsim(:,1);
        0;
        0;
        0;
        gsim(:,1)];
%initial covariance matrix. Should be pretty big, except for w
Pinit = diag([10 10 10 .1 .1 .1 .1 .1 .1 1 1 1]);
%Pinit = diag(ones(12,1));

%mass is known
mass = 10;

%process noise covariance matrix
%the only place where there should really be noise is gdot (since we are not
%including any dynamic model for it) and maybe a little on wdot for effects not
%modeled by the system, but they should be rather small, so for now lets
%ommit wdot noise and just put noise on G.

Q = diag([0 0 0 0 0 0 0 0 0 10 10 10]);

%H matrix
H = [0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1];

%Measurement covariance matrix
R = diag([.01 .01 .01 .01 .01 .01]);
% R = zeros(6);



%r contains x, P, and the nonzero section of the A matrix
global globAsym globDvec globM
globAsym = drv;
globDvec = dvec;
globM = m;
A = subs(drv,dvec,xinit);
A = subs(A,m,mass);
r = zeros((12+12*12),length(tsim));

%flag for recalculating A. If you do it too much, it slows performance way
%down
global rcA

r(:,1) = [xinit; reshape(Pinit,12*12,1)];
for i = 2:length(tsim)
    i
    rcA = 1;
    [t, rprop] = ode45(@EKF1prop,[0 tsim(i)-tsim(i-1)],r(:,i-1));
    rprop = rprop(end,:);
    xprop = rprop(1:12)';
    Pprop = reshape(rprop(13:12+12*12),12,12);
    
    %update step
    y = [wsim(:,i);
        gsim(:,i)];
    res = y - H*xprop;
    K = Pprop*H'*inv(H*Pprop*H'+R);
%     nk1 = norm(Pprop*H')
%     nk2 = norm(H*Pprop*H')
    nk = norm(K)
    xupd = xprop+K*res;
    Pupd = (eye(12)-K*H)*Pprop;
    A = subs(drv,[dvec;m],[xupd;mass]);
    %A = subs(A,m,mass);
    r(:,i) = [xupd; reshape(Pupd,12*12,1)];
end

%%
close all

II = r(1:3,:);
figure
plot(tsim,II(1,:))
hold all
plot(tsim,II(2,:));
plot(tsim,II(3,:));
%axis([0 tsim(end) -5 5])
title('MOI diagonal');

com = r(7:9,:);
figure
plot(tsim,com(1,:))
hold all
plot(tsim,com(2,:));
plot(tsim,com(3,:));
title('CoM location');

gB = r(10:12,:);
figure
plot(tsim,gB(1,:))
hold all
plot(tsim,gB(2,:));
plot(tsim,gB(3,:));
title('gravity vector');


