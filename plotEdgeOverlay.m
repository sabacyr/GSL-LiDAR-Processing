function [centers, edgeSets] = plotEdgeOverlay(ptCloud, targetPoints, titleText)
    figure
    pcshow(ptCloud.Location, ptCloud.Intensity, 'MarkerSize', 20)
    hold on
    xlim([518036.5 518038]); ylim([4972912.5 4972913.5]);
    clim([0 75])
    colormap gray
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(titleText)

    epsilon = 0.02;
    minpts = 10;
    labels = dbscan(targetPoints(:,1:2), epsilon, minpts);
    uniqueLabels = unique(labels);

    centers = [];       % Store [X Y Z] centers
    edgeSets = {};      % Store edge point groups (as cells)

    for k = 1:length(uniqueLabels)
        if uniqueLabels(k) == -1
            continue
        end

        clusterPoints = targetPoints(labels == uniqueLabels(k), :);
        XY = clusterPoints(:,1:2);

        try
            k_bnd = boundary(XY(:,1), XY(:,2), 0.9);
            edgePts = clusterPoints(k_bnd, 1:3);

            % Plot red outline
            plot3(edgePts(:,1), edgePts(:,2), edgePts(:,3), 'r-', 'LineWidth', 2);

            % Estimate center
            centerX = mean(edgePts(:,1));
            centerY = mean(edgePts(:,2));
            centerZ = mean(edgePts(:,3));

            centers = [centers; centerX, centerY, centerZ];
            edgeSets{end+1} = edgePts;

            % Mark center
            scatter3(centerX, centerY, centerZ, 60, 'g', 'filled');
            text(centerX, centerY, centerZ + 0.02, sprintf('C%d', k), 'Color', 'w', 'FontSize', 10);
        catch
            continue
        end
    end

    view(-160, 30)
end
