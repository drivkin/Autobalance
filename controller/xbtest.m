clear all;

xb1 = serial('COM5');
set(xb1,'BaudRate',38400);
set(xb1,'DataBits',8);
set(xb1,'StopBits',1);


xb2 = serial('COM6');
set(xb2,'BaudRate',38400);
set(xb2,'DataBits',8);
set(xb2,'StopBits',1);
fopen(xb1);
fopen(xb2);

for i = 1:100
    fwrite(xb2,i,'int32');
    pause(.01);
  %  a = fread(xb2,1,'int32')
end

a = fread(xb1,100,'int32')
%%
fclose(xb1);
fclose(xb2);