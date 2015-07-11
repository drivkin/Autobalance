%computes the derivative of the expression for w k plus 1 for use
%in the linearized PHI matrix for the discrete time EKF model
clear all

%these are global so they can be used by the function that propagates the
%EKF
global drv dvec ;


%sample time
ST = sym('ST');

%masses
bodyMass = sym('bodyMass');
m1 = sym('m1');
m2 = sym('m2');
m3 = sym('m3');

%mass positions
rm1 = sym('rm1',[3 1]);
rm2 = sym('rm2',[3 1]);
rm3 = sym('rm3',[3 1]);

%mass velocities
vm1 = sym('vm1',[3 1]);
vm2 = sym('vm2',[3 1]);
vm3 = sym('vm3',[3 1]);

%angular velocity expressed in body coordinates
w = sym('w',[3 1]);

%gravity vector expressed in body coordinates
g = sym('g',[3 1]);

%body MOI
Ibod = diag(sym('I',[1 3]));

%cg vector expressed in body coordinates
cg = sym('cg',[3 1]);


%mass MOIs and derivatives
Im1 = m1*skew(rm1).'*skew(rm1);
Im2 = m2*skew(rm2).'*skew(rm2);
Im3 = m3*skew(rm3).'*skew(rm3);

%time derivatives of mass MOIs
dIm1 = m1*(skew(vm1).'*skew(rm1)+skew(rm1).'*skew(vm1));
dIm2 = m2*(skew(vm2).'*skew(rm2)+skew(rm2).'*skew(vm2));
dIm3 = m3*(skew(vm3).'*skew(rm3)+skew(rm3).'*skew(vm3));

%Body gravity torque
TGB = cross(cg,bodyMass*g);

%Gravity torques on the masses
TGM1 = cross(rm1,m1*g);
TGM2 = cross(rm2,m2*g);
TGM3 = cross(rm3,m3*g);

%dynamics
term1 = -cross(-w,Ibod*w);
term2 = TGB+TGM1+TGM2+TGM3;
term3 = -(dIm1 + dIm2 + dIm3)*w;
term4 = -cross(w, (Im1+Im2+Im3)*w);
wdot = inv(Ibod)*(term1+term2); %trying without term 3 or 4 or mass MOIs

wkp1 = w +ST*wdot;

dvec = [w;
    rm1;
    rm2;
    rm3;
    vm1;
    vm2;
    vm3;
    g;
    Ibod(1,1);
    Ibod(2,2);
    Ibod(3,3);
    cg];

for i = 1: length(wkp1)
    for j = 1: length(dvec)
        j
        drvo(i,j) = diff(wkp1(i),dvec(j));
    end
end
dvec = [dvec;bodyMass;m1;m2;m3;ST];
drv = drvo;


%create command string to allow faster plugging in 
%first the cell array of strings that will allow us to tie numerical values
%to variable names
for i = 1:length(dvec)
    vS{i} = [char(dvec(i)) '=numVars(' num2str(i) ');'];
end

%now create the cell array of strings that will plug the numbers into
%the expression

for i = 1:size(drv,1)
    for j = 1:size(drv,2)
        fS{i,j} = ['M(' num2str(i) ',' num2str(j) ') =' char(drv(i,j)) ';'];
    end
end

save('commandStrings','fS','vS');


