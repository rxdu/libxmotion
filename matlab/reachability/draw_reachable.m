function draw_reachable(log_name)
    data = csvread(log_name);
    
    figure
    hold on;
    plot(data(:,1),data(:,2),'b.')
    plot(data(:,3),data(:,4),'r*')
end
