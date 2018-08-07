function draw_primitives(log_name)
    data = csvread(log_name);
    
    iidx = 2;
    x0 = data(1,iidx);
    y0 = data(1,iidx+1);
    v0 = data(1,iidx+2);
    theta0 = data(1,iidx+3);
    u0 = v0.*cos(theta0);
    v0 = v0.*sin(theta0);

    fidx = 6;
    x = data(:,fidx);
    y = data(:,fidx+1);
    v = data(:,fidx+2);
    theta = data(:,fidx+3);    
    u = v.*cos(theta);
    v = v.*sin(theta);
    
    figure
    axis equal
    hold on;
    %plot(x,y,'b.')
    quiver(x0,y0,u0,v0, 'r')
    quiver(x,y,u,v,'b')
    %plot(data(:,3),data(:,4),'r*')
end
