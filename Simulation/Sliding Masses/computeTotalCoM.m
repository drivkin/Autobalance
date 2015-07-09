function [CoMT] = computeTotalCoM(M,m,CoMB,rm)
%Compute the center of mass of body together with the translating masses

%M is mass of body
% m(1:3) is masses of the linear masses
%CoMB(1:3) is the center of mass of the body
%rm(1:9) is positions of the masses

%Mt is total mass of the system
%CoMT is the total center of mass

Mt = M + sum(m);

CoMT = M*CoMB + m(1)*rm(1:3) +m(2)*rm(4:6)+m(3)*rm(7:9);
CoMT = CoMT/Mt;
end

