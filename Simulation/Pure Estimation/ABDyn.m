function [ rDot ] = ABDyn(t, r )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

q = r(1:4)';
w = r(5:7);

%moment of inertia
I = diag([.1 .2 .3]);
invI = inv(I);
% total mass of system
m = 10;

% displacement vector of center of mass from center of rotation in body
% coordinates
com = [0.03;
       0.01;
        -.01];
    
%compute gravity vector in body coordinates
gInert = [0 0 -9.8]';
gBody = quat2dcm(q)*gInert;

% wdot = invI*(cross(com,m*gBody) - cross(w,I*w));
 wdot = invI*(skew(com)*m*gBody - skew(w)*I*w);


%qdot = qDotQG(w,q);
 qdum = [0 w'];
 qdot = 0.5*quatmultiply(q,qdum);
rDot = [qdot';wdot];


end

