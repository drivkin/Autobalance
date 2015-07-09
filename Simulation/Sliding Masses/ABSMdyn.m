function [ rDot ] = ABSMdyn( t,r )
%Compute dynamics for air bearing with three translating masses. The masses
%translate parallel to the body x,y, and z axes, but are not necessarily
%coinicident with the axes. The effects of dynamic translation of the
%masses on the rotational dynamics of the body are modeled.

%Extract the states from the vector containing them all
%Rotation quaternion between body frame and inertial frame. Both frames
%have origins at the center of rotation of the air bearing.
q = r(1:4)';

%Angular velocity of the body wrt inertial frame, expressed in body
%coordinates
w = r(5:7);

%positions of the sliding masses in body coordinates
rm1 = r(8:10);
rm2 = r(11:13);
rm3 = r(14:16);

%velocities of the sliding masses in body coordinates. Mass 1 is on the x,
%mass 2 on the y, mass 3 on the z.
vm1 = [r(17) 0 0]';
vm2 = [0 r(18) 0]';
vm3 = [0 0 r(19)]';

%accelerations of the sliding masses in body coordinates.
am1 = [r(20) 0 0]';
am2 = [0 r(21) 0]';
am3 = [0 0 r(22)]';


%Now the constants
global M m I comB;

%Compute intermediate values

%MOI matrices for the masses.
Im1 = m(1)*skew(rm1)'*skew(rm1);
Im2 = m(2)*skew(rm2)'*skew(rm2);
Im3 = m(3)*skew(rm3)'*skew(rm3);

%time derivatives of MOI matrices for the masses
dIm1 = m(1)*(skew(vm1)'*skew(rm1)+skew(rm1)'*skew(vm1));
dIm2 = m(2)*(skew(vm2)'*skew(rm2)+skew(rm2)'*skew(vm2));
dIm3 = m(3)*(skew(vm3)'*skew(rm3)+skew(rm3)'*skew(vm3));

%Angular component of the linear velocities and accelerations of the masses
a = rm1/norm(rm1);
N = eye(3) - a*a';
av1 = N*vm1/norm(rm1);
aa1 = N*am1/norm(rm1);

a = rm2/norm(rm2);
N = eye(3) - a*a';
av2 = N*vm2/norm(rm2);
aa2 = N*am2/norm(rm2);

a = rm3/norm(rm3);
N = eye(3) - a*a';
av3 = N*vm3/norm(rm3);
aa3 = N*am3/norm(rm3);

%Gravity vector in body coordinates
gInert = [0 0 -9.8]';
gBody = quat2dcm(q)*gInert;

%Body gravity torque
TGB = cross(comB,M*gBody);

%Gravity torques on the masses
TGM1 = cross(rm1,m(1)*gBody);
TGM2 = cross(rm2,m(2)*gBody);
TGM3 = cross(rm3,m(3)*gBody);

%now the derivatives

qdum = [0 w'];
qdot = 0.5*quatmultiply(q,qdum);

%this is the long one, and the derivation is not included here. The key
%idea to remember is that the torque exerted by the mass on the body is
%equal to the gravity torque on the mass minus the change in the masses
%angular momentum
x = inv(I+Im1+Im2+Im3);
y = TGB + TGM1 + TGM2 + TGM3;
z = cross(w,I*w);
xx = (dIm1*(w+av1)+dIm2*(w+av2)+dIm3*(w+av3));
yy = (Im1*aa1 + Im2*aa2 + Im3*aa3);
zz = (cross(w+av1,Im1*(w+av1))+cross(w+av2,Im2*(w+av2))+cross(w+av3,Im3*(w+av3)));
wdot = x*(y - z...
    -xx...
    -yy...
    -zz);


%putting it all back together
rDot = [qdot';
        wdot;
        vm1; %derivative of position is velocity
        vm2;
        vm3;
        am1(1); %derivative of velocity is acceleration
        am2(2);
        am3(3);
        0; %linear acceleration of the masses doesn't change, it's a control
        0;
        0];
end

