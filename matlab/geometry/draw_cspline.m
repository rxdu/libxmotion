function draw_cspline(log_name)
    data = csvread(log_name);
    plot(data(:,1), data(:,2));
end
