function [ xhat_out ] = ABEKFupdate(y)
% Currently, the EKF is based on a model without the sliding masses,
% although it does allow for translation of the center of mass. It probably
% won't do a great job, but I want to see how well it gets the sign of the
% components of the CM vector

persistent xhat P PHI;
persistent initFlag;

%global constants
global T M m I comB;

%symbolic stuff
global drv dvec;


if(isempty(initFlag))
%state is [I11;I22;I33;w;cg;g] where the first three are the diagonal terms
%of the MOI matrix, w is the angular momentum expressed in the body frame,
%cg is the center of gravity vector (in the body frame) and g is the
%gravity vector( in the body frame)

xhat = [I(1,1);
    I(2,2);
    I(3,3)
    y(1:3);
    comB;
    y(4:6)];
%initial covariance matrix. Should be pretty big, except for w
% currently I is considered known, so we put covariance to zero
P = diag([1 1 1 .1 .1 .1 .1 .1 .1 1 1 1]);
%Pinit = diag(ones(12,1));

PHI = diag(ones(12,1));
initFlag = 1;
end



%process noise covariance matrix
%the only place where there should really be noise is gdot (since we are not
%including any dynamic model for it) and on CM since it'll be changing

Q = T*diag([0 0 0 0 0 0 1 1 1 100 100 100]);

%H matrix
H = [0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1];

%Measurement covariance matrix
R = T*diag([.01 .01 .01 .01 .01 .01]);

%Begin filtering
%nonzero part of linearized PHI matrix
ap = subs(drv,dvec,[xhat;M;T]);
PHI(4:6,:) = ap;

%propagate x
x = xhat;
Iflt = diag(x(1:3));
w = x(4:6);
cg = x(7:9);
g = x(10:12);


xx = inv(Iflt);
wnext = w + T*xx*(M*skew(cg)*g-skew(w)*(Iflt*w));
xnext = x;
xnext(4:6) = wnext;
xhat = xnext;

%Propagate P
P = PHI*P*PHI'+Q;

%update step
K = P*H'*inv(H*P*H'+R);
res = y-H*xhat;

%update x
xhat = xhat+K*res;

%update P
P = (eye(12) - K*H)*P;

%output
xhat_out = xhat;


end

