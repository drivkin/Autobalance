function [ xhat_out ] = ABSMEKFupdate( y ,reset)
%This function implements the EKF update, where the dynamic model includes
%the sliding masses. However, the effects of mass velocity and acceleration
%on the dynamics of the testbed are ignored.
%The reason for writing this is that ignoring the
%movement of the masses seems to produce oscillatory behavior in the
%estimator when trying to control the CoM. We will know what the masses are
%doing pretty well, so lets include them in the model.

%y is measurements
%wbody (1:3)
%vmasses (4:12)
%gbody (13:15)



% xhat (Estimator state) is
%wBody (1:3)
%position of masses(4:12)
%velocity of masses(13:21)
%gBody (22:24) gravity vector in body frame
%Ibody (25:27) MOI of the body without the masses (diagonal for now)
%CoM Body (28:30) Center of mass of the body without the masses


persistent xhat P PHI;
persistent initFlag;

%global constants
global T M m I comB mpInit;

%symbolic stuff
global drv dvec wdfunc;

gyroNoise = [.1 .1 .1];
vMesNoise = 10^(-6)*[10 1 1 1 10 1 1 1 10];
gMesNoise = [.1 .1 .1];


if(isempty(initFlag))
    %set up initial guess
    
    %at the moment they're all perfect, this is a good place to start for
    %testing the estimator and controller. Once it works like this, we'll move
    %on and make it harder.
    xhat = [y(1:3);
        mpInit;
        0;0;0;0;0;0;0;0;0; %initial velocity of masses is zero
        y(13:15);
        I(1,1);
        I(2,2);
        I(3,3);
        comB];
    
    
    %initial guess covariance matrix
    pmGuessVar = 10^-4*[1 1 1 1 1 1 1 1 1];
    IguessVar = [2 2 2];
    comGuessNoise = 10^-2*[1 1 1];
    vGuessNoise = [0 0 0 0 0 0 0 0 0]; %we know the masses aren't sliding to start
    P = diag([gyroNoise pmGuessVar vGuessNoise gMesNoise IguessVar comGuessNoise]);
    szP = size(P)
    
    %set up the unchanging parts of the linearized PHI matrix for propagating
    %covariance
    PHI = eye(30);
    
    %submatrix of PHI that relfects the dependence of position on velocity
    smphi = T*eye(9);
    PHI(4:12,13:21) = smphi;
    
    initFlag = 1;
end

%if reset gets set high, reset the filter, keeping most of the estimate as
%it was
if(reset)
    xhat(1:3) = y(1:3);
    xhat(13:21) = y(4:12);
    xhat(22:24) = y(13:15);
    
    %might also need to do something to the covariance matrix
    %initial guess covariance matrix
    pmGuessVar = 10^-4*[1 1 1 1 1 1 1 1 1];
    IguessVar = [2 2 2];
    comGuessNoise = 10^-2*[1 1 1];
    vGuessNoise = [0 0 0 0 0 0 0 0 0]; %we know the masses aren't sliding to start
    P = diag([gyroNoise pmGuessVar vGuessNoise gMesNoise IguessVar comGuessNoise]);
end

%process noise covariance matrix
wPN = 10^-5*[1 1 1]; %pretty low because the model is good, but not too low because these dynamics do not account for velocity and acceleration of linear masses
pmPN = [0 0 0 0 0 0 0 0 0]; %none because it's just the integral of velocity
vmPN = 100*[1 1 1 1 1 1 1 1 1];%high because it's measured
gbodyPN = 100*[1 1 1]; %high because it's measured
IbodyPN = [ 0 0 0]; %because it's constant
combodyPN = [0 0 0]; %because it's constant

Q = T*diag([wPN pmPN vmPN gbodyPN IbodyPN combodyPN]);

%observation matrix
H = zeros(15,30);
H(1:3,1:3) = eye(3);%observing wbody
H(4:12,13:21) = eye(9);%observing vmasses
H(13:15,22:24) = eye(3); %observing gravity vector

%Measurement noise matrix
R = diag([gyroNoise vMesNoise gMesNoise]);

%Begin filtering

%propagate xhat

%repack
w = xhat(1:3);
rm1 = xhat(4:6);
rm2 = xhat(7:9);
rm3 = xhat(10:12);
vm1 = xhat(13:15);
vm2 = xhat(16:18);
vm3 = xhat(19:21);
gBody = xhat(22:24);
Ib = xhat(25:27);
cmBody = xhat(28:30);

%compute MOIs
Ib = diag(Ib);
Im1 = m(1)*skew(rm1)'*skew(rm1);
Im2 = m(2)*skew(rm2)'*skew(rm2);
Im3 = m(3)*skew(rm3)'*skew(rm3);
tMOI = Ib+Im1+Im2+Im3;

%time derivatives of mass MOIs
dIm1 = m(1)*(skew(vm1)'*skew(rm1)+skew(rm1)'*skew(vm1));
dIm2 = m(2)*(skew(vm2)'*skew(rm2)+skew(rm2)'*skew(vm2));
dIm3 = m(3)*(skew(vm3)'*skew(rm3)+skew(rm3)'*skew(vm3));

%Body gravity torque
TGB = cross(cmBody,M*gBody);

%Gravity torques on the masses
TGM1 = cross(rm1,m(1)*gBody);
TGM2 = cross(rm2,m(2)*gBody);
TGM3 = cross(rm3,m(3)*gBody);

%once again, don't forget we're ignoring any terms including velocity or
%acceleration of the masses
term1 = -cross(-w,Ib*w);
term2 = TGB+TGM1+TGM2+TGM3;
term3 = -(dIm1 + dIm2 + dIm3)*w;
term4 = -cross(w, (Im1+Im2+Im3)*w);

wdot = tMOI\(term1+term2+term3+term4);

%discretizing
wkp1 = w+T*wdot;
rmp1 = [rm1;rm2;rm3]+T*[vm1;vm2;vm3];

xhat(1:3) = wkp1;
xhat(4:12) = rmp1;

%propagate P

%nonzero part of linearized PHI matrix
%ap = subs(drv,dvec,[xhat;M;m;T]);
% varString = sprintf('%.0f,' , [xhat;M;m;T]');
% varString = varString(1:end-1)% strip final comma
%
% fstring =['ap = wdfunc(' varString ');'];
% eval(fstring);
% ap

ap = plugIn('commandStrings',[xhat;M;m;T]);
PHI(1:3,:) = ap;

P = PHI*P*PHI'+Q;

%update step
K = P*H'/(H*P*H'+R);
res = y-H*xhat;

%update x
xhat = xhat+K*res;

%update P
P = (eye(30) - K*H)*P;

%output
xhat_out = xhat;









end

