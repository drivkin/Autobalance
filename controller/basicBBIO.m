s = serial('COM4');
set(s,'BaudRate',38400);
set(s,'DataBits',8);
set(s,'Parity','even');
set(s,'StopBits',1);
fopen(s)
%%
%
% arr
% for i = 1:100
%     pause(.01);
%     arr = [4 i i i]';
%     fwrite(s,arr,'int32');
% end
setM1P = 1;
setM2P = 2;
setM3P = 3;
sleep = 4;
wake = 5;
tellPeriods = 6;



mcurr = 2;


arr = [0 0 0 0];
mspeeds = arr;
while(1)
    k = getkey('non-ascii');
    if(strcmpi(k,'w'))
        arr = [wake 0 0 0];
        mspeeds = arr;
        fwrite(s,arr,'int32');
    end
    
    if(strcmpi(k,'s'))
        arr = [sleep 0 0 0];
        mspeeds = arr;
        fwrite(s,arr,'int32');
    end
    
    if(strcmpi(k,'1'))
        mcurr = 2;
    end
    
    if(strcmpi(k,'2'))
        mcurr = 3;
    end
    
    if(strcmpi(k,'3'))
        mcurr = 4;
    end
    
    
    if(strcmpi(k,'a'))
        cs = mspeeds(mcurr);
        cs = cs - 100;
        if(cs < -1000)
            cs = -1000;
        end
        mspeeds(mcurr) = cs;
        arr(mcurr) = speedToPeriod(cs);
        arr(1) = mcurr-1;
        fwrite(s,arr,'int32');
    end
    
    
    if(strcmpi(k,'d'))
        cs = mspeeds(mcurr);
        cs = cs + 100;
        if(cs > 1000)
            cs = 1000;
        end
        mspeeds(mcurr) = cs;
        arr(mcurr) = speedToPeriod(cs);
        arr(1) = mcurr-1;
        fwrite(s,arr,'int32');
    end
    
    if(strcmpi(k,'t'))
        arr(1) = tellPeriods;
        fwrite(s,arr,'int32');
    end
    
    if(strcmpi(k,'x'))
        fclose(s);
        break;
    end     
    
    
end









