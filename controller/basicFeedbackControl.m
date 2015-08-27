s = serial('COM4');
set(s,'BaudRate',38400);
set(s,'DataBits',8);
set(s,'Parity','even');
set(s,'StopBits',1);
fopen(s)

%% read uart, print out what it's giving, and then send some command

while(1)
    fwrite(s,generateBBBCommand('tell_sens',0),'int32');
    A = fread(s,3,'int32')
end

%%
fclose(s);