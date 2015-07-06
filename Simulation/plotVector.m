function [ ] = plotVector(t,v,name)
%Plots t vs the entries of vector v. V should be arranged so that the rows
%are different vector entries and the columns are the time points. t and
%name can be null matrices.
if(~isempty(t))
    plot(t,v(1,:));
    hold all
    for i = 2:size(v,1)
        plot(t,v(i,:));
    end
else
    plot(v(1,:));
    hold all
    for i = 2:size(v,1)
        plot(v(i,:));
    end
end

if(~isempty(name))
    for i =1:size(v,1)
        args{i} = [name '_' num2str(i)];
    end
else
     for i =1:size(v,1)
        args{i} = [num2str(i)];
     end
end
legend(args);

end

