%% Load the file
load OG1to4.mat

%% Scanner global position (from UTM + elevation + instrument height)
X0 = 518035.91;     % meters
Y0 = 4972915.88;    % meters
Z0 = 36.867;        % meters (116 ft + 1.51 m)

%% Azimuth angle in degrees (clockwise from North)
azimuth_deg = 0;
azimuth_rad = deg2rad(azimuth_deg);

%% Rotation matrix for yaw only
R = [cos(azimuth_rad), -sin(azimuth_rad), 0;
     sin(azimuth_rad),  cos(azimuth_rad), 0;
     0,                 0,                1];

%% Transform  
XYZ1 = [Scan1.VarName1, Scan1.VarName2, Scan1.VarName3]';
XYZ1_rot = R * XYZ1;
XYZ1_global = XYZ1_rot + [X0; Y0; Z0];

XYZ2 = [Scan2.VarName1, Scan2.VarName2, Scan2.VarName3]';
XYZ2_rot = R * XYZ2;
XYZ2_global = XYZ2_rot + [X0; Y0; Z0];

XYZ3 = [Scan3.VarName1, Scan3.VarName2, Scan3.VarName3]';
XYZ3_rot = R * XYZ3;
XYZ3_global = XYZ3_rot + [X0; Y0; Z0];

XYZ4 = [Scan4.VarName1, Scan4.VarName2, Scan4.VarName3]';
XYZ4_rot = R * XYZ4;
XYZ4_global = XYZ4_rot + [X0; Y0; Z0];

% Transpose back to Nx3 matrices
XYZ1_global = XYZ1_global';
XYZ2_global = XYZ2_global';
XYZ3_global = XYZ3_global';
XYZ4_global = XYZ4_global';

figure(1)
scatter3(XYZ1_global(:,1), XYZ1_global(:,2), XYZ1_global(:,3), 1, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scan 1');
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(2)
scatter3(XYZ2_global(:,1), XYZ2_global(:,2), XYZ2_global(:,3), 1, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis tight
title('Scan 2');
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(3)
scatter3(XYZ3_global(:,1), XYZ3_global(:,2), XYZ3_global(:,3), 1, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis tight
title('Scan 3');
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(4)
scatter3(XYZ4_global(:,1), XYZ4_global(:,2), XYZ4_global(:,3), 1, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis tight
title('Scan 4');
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

%% Color by intensity
I1 = Scan1.VarName4;
I2 = Scan2.VarName4;
I3 = Scan3.VarName4;
I4 = Scan4.VarName4;

Z1 = XYZ1_global(:,3);
Z2 = XYZ2_global(:,3);
Z3 = XYZ3_global(:,3);
Z4 = XYZ4_global(:,3);

figure(5)
scatter3(XYZ1_global(:,1), XYZ1_global(:,2), XYZ1_global(:,3), 3, I1, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scan 1 - Colored by Intensity');
colorbar; clim([0 75]); view(-160, 15)
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(6)
scatter3(XYZ2_global(:,1), XYZ2_global(:,2), XYZ2_global(:,3), 3, I2, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scan 2 - Colored by Intensity');
colorbar; clim([0 75]); view(-140, 30)
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(7)
scatter3(XYZ3_global(:,1), XYZ3_global(:,2), XYZ3_global(:,3), 3, I3, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scan 3 - Colored by Intensity');
colorbar; clim([0 75]); view(-140, 30)
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(8)
scatter3(XYZ4_global(:,1), XYZ4_global(:,2), XYZ4_global(:,3), 3, I4, '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scan 4 - Colored by Intensity');
colorbar; clim([0 75]); view(-140, 30)
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

%% Turn into point cloud

ptCloud1 = pointCloud(XYZ1_global, 'Intensity', I1);
ptCloud2 = pointCloud(XYZ2_global, 'Intensity', I2);
ptCloud3 = pointCloud(XYZ3_global, 'Intensity', I3);
ptCloud4 = pointCloud(XYZ4_global, 'Intensity', I4);

figure(9)
pcshow(ptCloud1)
title('Scan 1 - Point Cloud')
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]); clim([0 75]) 
view(-160, 15)
xlabel('X'); ylabel('Y'); zlabel('Z');

figure(10)
pcshow(ptCloud2)
title('Scan 2 - Point Cloud')
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]); clim([0 75]) 
view(-160, 15)
xlabel('X'); ylabel('Y'); zlabel('Z');

figure(11)
pcshow(ptCloud3)
title('Scan 3 - Point Cloud')
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]); clim([0 75]) 
view(-160, 15)
xlabel('X'); ylabel('Y'); zlabel('Z');

figure(12)
pcshow(ptCloud4)
title('Scan 4 - Point Cloud')
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]); clim([0 75]) 
view(-160, 15)
xlabel('X'); ylabel('Y'); zlabel('Z');

%% All scans

AllPoints = [XYZ1_global; XYZ2_global; XYZ3_global; XYZ4_global];

AllIntensities = [I1; I2; I3; I4];

ptCloudAll = pointCloud(AllPoints, 'Intensity', AllIntensities);

figure(13)
pcshow(ptCloudAll.Location, ptCloudAll.Intensity)
colorbar; clim([0 50]) 
title('Combined Point Cloud - All Scans')
xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

%% Find objects in each scan
mask1 = I1 > 70;
mask2 = I2 > 70;
mask3 = I3 > 70;
mask4 = I4 > 70;

target1 = [XYZ1_global(mask1, :) I1(mask1)];
target2 = [XYZ2_global(mask2, :) I2(mask2)];
target3 = [XYZ3_global(mask3, :) I3(mask3)];
target4 = [XYZ4_global(mask4, :) I4(mask4)];

targetAll = [target1; target2; target3; target4];

writematrix(target1, 'target1.txt', 'Delimiter', 'tab');
writematrix(target2, 'target2.txt', 'Delimiter', 'tab');
writematrix(target3, 'target3.txt', 'Delimiter', 'tab');
writematrix(target4, 'target4.txt', 'Delimiter', 'tab');

%target4cleaned = table2array(target4cleaned)

% figure(14)
% scatter3(target1cleaned(:,1), target1cleaned(:,2), target1cleaned(:,3), 10, target1cleaned(:,4), 'filled');
% xlabel('X'); ylabel('Y'); zlabel('Z');
% colorbar; clim([100 max(target1cleaned(:,4))])
% title('High-Intensity Targets (>100)');
% xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);
% 
% figure(15)
% pcshow(ptCloudAll.Location, ptCloudAll.Intensity)
% colormap gray
% colorbar
% title('Full Scan with High-Intensity Targets Overlaid')
% hold on
% scatter3(targetAll(:,1), targetAll(:,2), targetAll(:,3), 15, 'r', 'filled');
% xlim([518036.5 518038.5]); ylim([4972912.5 4972913.5]);

figure(16)
pcshow(ptCloud1.Location, ptCloud1.Intensity)
colormap gray
hold on
scatter3(target1cleaned(:,1), target1cleaned(:,2), target1cleaned(:,3), 15, 'r', 'filled')
title('Scan 1 with High-Intensity Targets')
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
view(-160, 15)

figure(17)
pcshow(ptCloud2.Location, ptCloud2.Intensity)
colormap gray
hold on
scatter3(target2cleaned(:,1), target2cleaned(:,2), target2cleaned(:,3), 15, 'r', 'filled')
title('Scan 2 with High-Intensity Targets')
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
view(-160, 15)

figure(18)
pcshow(ptCloud3.Location, ptCloud3.Intensity)
colormap gray
hold on
scatter3(target3cleaned(:,1), target3cleaned(:,2), target3cleaned(:,3), 15, 'r', 'filled')
title('Scan 3 with High-Intensity Targets')
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
view(-160, 15)

figure(19)
pcshow(ptCloud4.Location, ptCloud4.Intensity)
colormap gray
hold on
scatter3(target4cleaned(:,1), target4cleaned(:,2), target4cleaned(:,3), 15, 'r', 'filled')
title('Scan 4 with High-Intensity Targets')
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
view(-160, 15)

%% Animation
filename = 'FourScans_WithTargets.gif';

fig = figure;
set(gcf, 'Color', 'k')
axis tight
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
colormap gray
hold on

% --- Step 1: Scan 1 ---
pcshow(ptCloud1.Location, ptCloud1.Intensity, 'MarkerSize', 10);
hold on
scatter3(target1cleaned(:,1), target1cleaned(:,2), target1cleaned(:,3), 15, 'r', 'filled');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
clim([0 75])
view(-160, 30)
title('Scan 1', 'Color', 'w');
xlabel('X'); ylabel('Y'); zlabel('Z');
drawnow
frame = getframe(fig);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);
imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 1);

% --- Step 2: Scan 2 ---
clf
pcshow(ptCloud2.Location, ptCloud2.Intensity, 'MarkerSize', 10);
hold on
scatter3(target2cleaned(:,1), target2cleaned(:,2), target2cleaned(:,3), 15, 'r', 'filled');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
clim([0 75])
view(-160, 30)
title('Scan 2', 'Color', 'w');
xlabel('X'); ylabel('Y'); zlabel('Z');
drawnow
frame = getframe(fig);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);
imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1);

% --- Step 3: Scan 3 ---
clf
pcshow(ptCloud3.Location, ptCloud3.Intensity, 'MarkerSize', 10);
hold on
scatter3(target3cleaned(:,1), target3cleaned(:,2), target3cleaned(:,3), 15, 'r', 'filled');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
clim([0 75])
view(-160, 30)
title('Scan 3', 'Color', 'w');
xlabel('X'); ylabel('Y'); zlabel('Z');
drawnow
frame = getframe(fig);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);
imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1);

% --- Step 4: Scan 4 ---
clf
pcshow(ptCloud4.Location, ptCloud4.Intensity, 'MarkerSize', 10);
hold on
scatter3(target4cleaned(:,1), target4cleaned(:,2), target4cleaned(:,3), 15, 'r', 'filled');
xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
clim([0 75])
view(-160, 30)
title('Scan 4', 'Color', 'w');
xlabel('X'); ylabel('Y'); zlabel('Z');
drawnow
frame = getframe(fig);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);
imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1);

%% Track center, edge
[centers1, edgeSets1] = plotEdgeOverlay(ptCloud1, target1cleaned, 'Scan 1');
[centers2, edgeSets2] = plotEdgeOverlay(ptCloud2, target2cleaned, 'Scan 2');
[centers3, edgeSets3] = plotEdgeOverlay(ptCloud3, target3cleaned, 'Scan 3');
[centers4, edgeSets4] = plotEdgeOverlay(ptCloud4, target4cleaned, 'Scan 4');

%% Track displacement
disp_Tar1 = zeros(3, 6);  % [ΔX, ΔY, ΔZ, totalDisplacement, fromScan, toScan]
disp_Tar2 = zeros(3, 6);

for i = 1:3
    delta1 = centerAll_Tar1(i+1,:) - centerAll_Tar1(i,:);
    delta2 = centerAll_Tar2(i+1,:) - centerAll_Tar2(i,:);

    totalDist1 = norm(delta1);
    totalDist2 = norm(delta2);

    disp_Tar1(i, :) = [delta1, totalDist1, i, i+1];
    disp_Tar2(i, :) = [delta2, totalDist2, i, i+1];
end