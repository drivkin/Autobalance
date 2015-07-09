function [ qdot ] = qDotQG( w,q )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q = quatnormalize(q);

A = [0 w(3) -w(2) w(1);
        -w(3) 0 w(1) w(2);
        w(2) -w(1) 0 w(3);
        -w(1) -w(2) -w(3) 0];

qdot = 0.5*A*q';
qdot = qdot';


end

