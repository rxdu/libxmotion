close all; clear; clc;

%log_name = "/home/rdu/monte_carlo_sim.20190120231844.data"; % v in one
%direction
% log_name = "/home/rdu/monte_carlo_sim.20190121060351.data";
%log_name = "/home/rdu/monte_carlo_sim.20190121080417.data"; % offset by 20
% log_name = "/home/rdu/monte_carlo_sim.20190121115441.data";

log_name = "/home/rdu/monte_carlo_sim.20190122042127.data";
%log_name = "/home/rdu/monte_carlo_sim.20190203065130.data";

data = csvread(log_name);
n_mcs = size(data, 1)

y = data(:,5);

figure
plot(data(:,1), data(:,2), 'c*')
pbaspect([1 1 1])

figure
plot(data(:,3), data(:,4), 'g+')
pbaspect([1 1 1])

figure
n_bins = 50;
[cnt_y, cen_y] = hist(y, n_bins);
pdf_y = (cnt_y / n_mcs) / ((max(y) - min(y))/n_bins);
stairs(cen_y, pdf_y, 'LineWidth', 1)
title("PDF")

% dx = (max(y) - min(y))/n_bins;
% sum(pdf_y*dx)

% cdf = zeros(n_bins);
% for i = 1:n_bins
%     cdf(i) = sum(pdf_y(1:i) * dx);
% end
% figure
% plot(cdf)

[parmhat,parmci] = lognfit(pdf_y)

CDF = ([1:n_mcs]-0.5) / n_mcs;
figure
stairs(sort(y), CDF, 'LineWidth', 1)
title("CDF")