function [ com_arr ] = generateBBBCommand(com_string, arg)
%This will generate an array of integers that you should send over UART to
%the bbb to get it to do something. Commands are always arrays two
%integers long. 

% 'echo' -- echoes back the command string
% 'sleep' -- puts all motors to sleep
% 'wake' -- wakes all motors
% 'm1speed' -- sets speed of m1 to arg. Max arg is 1000, min arg is -1000
% (reverse)
% 'm2speed' 
% 'm3speed'
% 'tell_mp' -- echoes back motor periods (in ms)
% 'tell_sens' -- echoes back two ADC values (MAV filtered) and limit switch state
coms = {'echo';
        'm1speed';
        'm2speed';
        'm3speed';
        'sleep';
        'wake';
        'tell_mp';
        'tell_sens'};

com_arr = [0 speedToPeriod(arg)];

for i = 1:length(coms)
    if(strcmpi(coms{i},com_string))
        com_arr(1) = i-1;
        break;
    end
end
end

