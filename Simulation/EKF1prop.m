function [ rDot ] = EKF1prop(t, r )
%propagation of state and covariance for EKF1 

global globAsym
global globDvec
global globM
global rcA
global A
%process covariance matrix
Q = diag([0 0 0 0 0 0 0 0 0 100 100 100]);
m = 10;

x = r(1:12);
P = reshape(r(13:12+12*12),12,12);
%np= norm(P)

if(rcA)
    ap = subs(globAsym,[globDvec;globM],[x;m]);
    A = zeros(12,12);
    A(4:6,:) = ap;
end

Pdot = A*P+P*A'+Q;
%npdot = norm(Pdot)

I = diag(x(1:3));
w = x(4:6);
cg = x(7:9);
g = x(10:12);

wdot = inv(I)*(m*skew(cg)*g-skew(w)*(I*w));
%wdot = inv(I)*(m*skew(cg)*g);

xdot = zeros(12,1);
xdot(4:6) = wdot;

rDot = [xdot;reshape(Pdot,12*12,1)];
rcA = 0;
end