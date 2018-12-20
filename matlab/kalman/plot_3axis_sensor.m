function plot_3axis_sensor(utime, data, plotlabel)

    figure
    hold on
    plot(utime/1000, data(:,1), 'r')
    plot(utime/1000, data(:,2), 'g')
    plot(utime/1000, data(:,3), 'b')
    hold off
    title(plotlabel)
    xlabel('time(s)')
    legend('x','y','z')

end

