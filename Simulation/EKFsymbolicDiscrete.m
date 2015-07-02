%computes the nonzero part of the linearized PHI matrix for the discrete time EKF model
clear all

%sample time
ST = sym('ST');

I = diag(sym('I',[1 3]));

%angular velocity expressed in body coordinates
w = sym('w',[3 1]);

%gravity vector expressed in body coordinates
g = sym('g',[3 1]);

%cg vector expressed in body coordinates
cg = sym('cg',[3 1]);

%mass
m = sym('m');

%omega at time k+1
wkp1 = sym('wkp1',[3 1]);

wkp1 = w + ST*inv(I)*(m*skew(cg)*g - skew(w)*(I*w));
%wdot = inv(I)*(m*skew(cg)*g);

dvec = [I(1,1);
        I(2,2);
        I(3,3);
        w;
        cg;
        g];
for i = 1: length(wkp1)    
    for j = 1: length(dvec)
        drv(i,j) = diff(wkp1(i),dvec(j));
    end
end
