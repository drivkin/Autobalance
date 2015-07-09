function [ xhat ] = ABEFKupdate(y)
%

%If all is as I think it is, the exact value of I isn't important for the
%sign controller I want to implement. As such, just use I close to what it
%is without sliding masses. We won't be estimating it now. We'll still keep
%it as a state though, just with zero initial covariance

persistent xhat;
persistent initFlag;

if(isempty(initFlag))
%state is [I11;I22;I33;w;cg;g] where the first three are the diagonal terms
%of the MOI matrix, w is the angular momentum expressed in the body frame,
%cg is the center of gravity vector (in the body frame) and g is the
%gravity vector( in the body frame)
xinit = [1;
    2;
    3;
    y(1:3);
    0;
    0;
    0;
    y(4:6)];
%initial covariance matrix. Should be pretty big, except for w
% currently I is considered known, so we put covariance to zero
Pinit = diag([0 0 0 .1 .1 .1 .1 .1 .1 1 1 1]);
%Pinit = diag(ones(12,1));
xupd = xinit;
xprop = xinit;
Pupd = Pinit;
Pprop = Pinit;

PHI = diag(ones(12,1));
initFlag = 1;
end


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







end

