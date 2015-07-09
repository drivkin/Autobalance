function [ qEq ] = computeEquilibriumPosition(CM)
%Compute an equilibrium position (i.e. one where gravity torque is zero)
%given a center of mass. Note there are an infinite number of equilibrium
%positions. Also, this will not work if the z component of CM is 0.

%CM is center of mass
%qEq is an equilibrium quaterion

%first we computed a DCM

%the unit vector of the inertial frame in body coordinates for the
%equilibrium position (in equilibrium, the CM is parallel to the gravity
%vector, which lies on the negative Z axis of the inertial frame)
zvec = -CM/norm(CM);

%compute an arbitrary vector perpendicular to zvec. It is arbitrary because
%there are an infinite number of equilibrium positions. Say the x and y
%coordinates of this vector are 1, find the z coordinate

a = (-zvec(1) - zvec(2))/zvec(3);

xvec = [1;1;a];
xvec = xvec/norm(xvec);

yvec = cross(zvec,xvec);

DCM = [xvec yvec zvec];

qEq = dcm2quat(DCM);

end

