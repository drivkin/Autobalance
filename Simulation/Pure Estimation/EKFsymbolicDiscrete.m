%computes the nonzero part of the linearized PHI matrix for the discrete time EKF model
%clear all

%these are global so they can be used by the function that propagates the
%EKF
global drv dvec;


%sample time
ST = sym('ST');

Isym = diag(sym('I',[1 3]));

%angular velocity expressed in body coordinates
w = sym('w',[3 1]);

%gravity vector expressed in body coordinates
g = sym('g',[3 1]);

%cg vector expressed in body coordinates
cg = sym('cg',[3 1]);

%mass
mass = sym('m');

%omega at time k+1
wkp1 = sym('wkp1',[3 1]);

wkp1 = w + ST*inv(Isym)*(mass*skew(cg)*g - skew(w)*(Isym*w));
%wdot = inv(I)*(m*skew(cg)*g);

dvec = [Isym(1,1);
        Isym(2,2);
        Isym(3,3);
        w;
        cg;
        g];

for i = 1: length(wkp1)    
    for j = 1: length(dvec)
        j
        drvo(i,j) = diff(wkp1(i),dvec(j));
    end
end
dvec = [dvec;mass;ST];
drv = drvo;