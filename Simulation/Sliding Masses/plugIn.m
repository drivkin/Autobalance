function [ M ] = plugIn(CSPath,numVars)

%This function plugs a numerical variables into pre-computed control
%strings. This is done to allow fast plugging into a symbolic equation,
%because the subs function is too damn slow. The command strings should be
%generated from the symbolic equations. The file that the path points to
%should have two cell arrays in it:

%vS contains strings which assign numeric values to variables
%fS contains strings that do the plugging in

%inputs
%CSpath = path where the cell arrays of command strings can be found
%numVars = vector of numeric variables that you want to plug in for the
%symbolic variables

%output 
%M = numeric matrix

persistent initFlag2 fS vS;


%if it's the first call, we have to compute all the strings that will let
%us plug stuff in quickly.
if(isempty(initFlag2))
    initFlag2 =1
    load(CSPath);
end

%assign variables
for i =1:length(numVars)
    eval(vS{i});
end

%plug in
M = zeros(size(fS));

for i = 1:size(fS,1)
    for j = 1:size(fS,2)
        eval(fS{i,j});
    end
end

end
            
    


