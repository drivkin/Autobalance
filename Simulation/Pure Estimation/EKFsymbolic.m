
%for now, assume MOI is diagonal
I = diag(sym('I',[1 3]));


%angular velocity expressed in body coordinates
w = sym('w',[3 1]);

%gravity vector expressed in body coordinates
g = sym('g',[3 1]);

%cg vector expressed in body coordinates
cg = sym('cg',[3 1]);

%mass
m = sym('m');

wdot = sym('wdot',[3 1]);

wdot = inv(I)*(m*skew(cg)*g - skew(w)*(I*w));
%wdot = inv(I)*(m*skew(cg)*g);

dvec = [I(1,1);
        I(2,2);
        I(3,3);
        w;
        cg;
        g];
for i = 1: length(wdot)    
    for j = 1: length(dvec)
        drv(i,j) = diff(wdot(i),dvec(j));
    end
end

% %%
% %lets see what we can do with substitution
% 
% dvecNum = [1;1;1;1;1;1;1;1;1;1;1;1];
% 
% a= subs(drv,dvec,dvecNum);
