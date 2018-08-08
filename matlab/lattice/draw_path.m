function draw_primitives_with_path(log_name, path_len)
    data = csvread(log_name);
    
    entry_num = size(data);
    mp_num = entry_num(1)/path_len
    
    % find initial states
    iidx = 2;
    x0 = data(1,iidx);
    y0 = data(1,iidx+1);
    v0 = data(1,iidx+2);
    theta0 = data(1,iidx+3);
    u0 = v0.*cos(theta0);
    v0 = v0.*sin(theta0);

    % find final states
    fidx = 6;
    xf = zeros(mp_num,1);
    yf = zeros(mp_num,1);
    vf = zeros(mp_num,1);
    thetaf = zeros(mp_num,1);
    uf = zeros(mp_num,1);
    vf = zeros(mp_num,1);
    for i = 1:mp_num
        xf(i) = data(i*path_len,fidx);
        yf(i) = data(i*path_len,fidx+1);
        vf(i) = data(i*path_len,fidx+2);
        thetaf(i) = data(i*path_len,fidx+3);            
    end
    uf = vf.*cos(thetaf);
    vf = vf.*sin(thetaf);
    
    figure
    axis equal
    hold on;
    quiver(x0,y0,u0,v0, 'r')
    quiver(xf,yf,uf,vf,'r')
    
    % draw paths
    for i = 1:mp_num
        path = [];
        for j = 1:path_len
            path = [path; [data((i-1)*path_len + j,fidx), data((i-1)*path_len + j,fidx+1)]];
        end
        plot(path(:,1),path(:,2),'b')
    end
end
