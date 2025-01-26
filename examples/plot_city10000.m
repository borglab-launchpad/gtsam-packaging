clear;

gt = dlmread('Data/ISAM2_GT_city10000.txt');

eh_poses = dlmread('../build/examples/ISAM2_city10000.txt');

h_poses = dlmread('../build/examples/HybridISAM_city10000.txt');

% Plot the same number of GT poses as estimated ones
% gt = gt(1:size(eh_poses, 1), :);
gt = gt(1:size(h_poses, 1), :);
eh_poses = eh_poses(1:size(h_poses, 1), :);


figure(1)
hold on;
axis equal;
axis([-65 65 -75 60])
plot(gt(:,1), gt(:,2), '--', 'LineWidth', 4, 'color', [0.1 0.7 0.1 0.5]);
% hold off;

% figure(2)
% hold on;
% axis equal;
% axis([-65 65 -75 60])
plot(eh_poses(:,1), eh_poses(:,2), '-', 'LineWidth', 2, 'color', [0.9 0.1 0. 0.4]);
plot(h_poses(:,1), h_poses(:,2), '-', 'LineWidth', 2, 'color', [0.1 0.1 0.9 0.4]);
hold off;
