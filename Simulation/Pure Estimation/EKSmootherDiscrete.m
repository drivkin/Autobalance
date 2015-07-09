%The smoother

%first run the filter
EKFdiscrete;

%%
N = length(tsim)
xsmooth = zeros(size(xupd));
xsmooth(:,end) = xupd(:,end);
Psmooth = zeros(size(Pupd));
PHI = diag(ones(12,1));
for j = 1:N-1 
    i = N - j;
    i
    ap = subs(drv,[dvec;m;ST],[xsmooth(:,i+1);mass;T]);
    PHI(4:6,:) = ap;
    
    A = Pupd(:,:,i)*PHI*inv(Pprop(:,:,i+1));
    xsmooth(:,i) = xupd(:,i)+A*(xsmooth(:,i+1)-xprop(:,i+1));
    Psmooth(:,:,i) = Pupd(:,:,i)+A*(Psmooth(:,:,i+1)-Pprop(:,:,i))*A';
end
    
%%
figure
plotVector(tsim,xsmooth(1:3,:),'smoothed I');

figure
plotVector(tsim,xsmooth(7:9,:),'smoothed COM');