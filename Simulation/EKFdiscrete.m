% This script implements the ekf on the discretized model

clc
%compute nonzero part of A matrix analytically
EKFsymbolicDiscrete;

T = .01; %sample time

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
xinit = [.1;
    .2;
    .3;
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

Q = T*diag([0 0 0 0 0 0 0 0 0 100 100 100]);

%H matrix
H = [0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1];

%Measurement covariance matrix
R = T*diag([.01 .01 .01 .01 .01 .01]);

xprop = zeros(12,length(tsim));
xupd = zeros(12,length(tsim));
Pprop = zeros(12,12,length(tsim));
Pupd = zeros(12,12,length(tsim));
xupd(:,1) = xinit;
xprop(:,1) = xinit;
Pupd(:,:,1) = Pinit;
Pprop(:,:,1) = Pinit;

PHI = diag(ones(12,1));
PHI_t = zeros(12,12,length(tsim));
%the filtering part
for i = 2:length(t)
    ap = subs(drv,[dvec;m;ST],[xupd(:,i-1);mass;T]);
    PHI(4:6,:) = ap;
    PHI_t(:,:,i-1) = PHI;
    nPHI = norm(PHI)
    
    %x propagation
    x = xupd(:,i-1);
    I = diag(x(1:3));
    w = x(4:6);
    cg = x(7:9);
    g = x(10:12);
    
    wnext = w + T*inv(I)*(mass*skew(cg)*g-skew(w)*(I*w));
    xnext = x;
    xnext(4:6) = wnext;
    xprop(:,i) = xnext;
    
    %P propagation
    P = Pupd(:,:,i-1);
    Pnext = PHI*P*PHI' + Q;
    Pprop(:,:,i) = Pnext;
    nP = norm(Pnext)
    
    %update step
    y = [wsim(:,i);
        gsim(:,i)];
    K = Pnext*H'*inv(H*Pnext*H' + R);
    nk = norm(K)
    xupd(:,i) = xnext+K*(y-H*xnext);
    Pupd(:,:,i) = (eye(12) - K*H)*Pnext;   
    nPupd = norm(Pupd(:,:,i))
end
%%
close all
r = xupd;
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
