function [ period ] = speedToPeriod( speed )
%Take a speed (in steps/second) and convert to a period (in ms). Positive
%speed gives positive period, negative speed gives negative period, zero
%speed gives zero period

if(speed ~= 0)
    period = round((1/speed)*1000);
else
    period = 0;
end


end

