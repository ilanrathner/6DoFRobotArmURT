function plotReachableWorkspaceCombos(linkLengths)
    % Plots reachable workspace of a 6-DOF robotic arm with toggles for 3 joint freedom combos.

    %% Joint limits (degrees)
    jointLimitsDeg = [
        -180, 180;
        -165, 165;
        -165, 165;
        -180, 180;
        -90,   90;
        -180, 180 %don't need this because don't need to rotate theta6 
                    % because everything will stay in the same position even if it rotates
    ];

    %% Resolution Combo A:  Joints 1–2 Free(degrees)
    AcoarseResDeg = 45;
    AfineResDeg = 20;

    %% Resolution Combo B: Joints 1–3 Free(degrees)
    BcoarseResDeg = 45;
    BfineResDeg = 20;

    %% Resolution Combo C: Joints 4–6 Free(degrees)
    CcoarseResDeg = 45;
    CfineResDeg = 30;

    %% Convert to radians
    jointLimitsRad = jointLimitsDeg * pi / 180;

    AcoarseResRad = deg2rad(AcoarseResDeg);
    AfineResRad   = deg2rad(AfineResDeg);

    BcoarseResRad = deg2rad(BcoarseResDeg);
    BfineResRad   = deg2rad(BfineResDeg);

    CcoarseResRad = deg2rad(CcoarseResDeg);
    CfineResRad   = deg2rad(CfineResDeg);

    %% Set up figure
    fig = figure('Name','Reachable Workspace with Toggle Controls');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, 'Reachable Workspace of 6-DOF Arm');

    %% Plot each combo
    scatterHandles = struct();

    scatterHandles.A = plotComboA(linkLengths, jointLimitsRad, AcoarseResRad, AfineResRad, ax);
    fprintf("A done");
    scatterHandles.B = plotComboB(linkLengths, jointLimitsRad, BcoarseResRad, BfineResRad, ax);
    fprintf("B done");
    scatterHandles.C = plotComboC(linkLengths, jointLimitsRad, CcoarseResRad, CfineResRad, ax, fig);
    fprintf("C done");

    %% Add toggles
    uicontrol('Style', 'checkbox', 'String', 'Show Combo A', ...
        'Position', [20, 20, 120, 20], 'Value', 1, ...
        'Callback', @(src,~) toggleVisibility(scatterHandles.A, src.Value));

    uicontrol('Style', 'checkbox', 'String', 'Show Combo B', ...
        'Position', [150, 20, 120, 20], 'Value', 1, ...
        'Callback', @(src,~) toggleVisibility(scatterHandles.B, src.Value));

    legend({'Combo A: Joints 1–2 free', ...
            'Combo B: Joints 1–3 free', ...
            'Combo C: Joints 4–6 free @ wrist poses'}, ...
           'Location', 'bestoutside');
end

%% === Helper to Toggle Visibility ===
function toggleVisibility(h, visibleFlag)
    if visibleFlag
        set(h, 'Visible', 'on');
    else
        set(h, 'Visible', 'off');
    end
end

%% === Combo A: Joints 1–2 Free ===
function h = plotComboA(linkLengths, jointLimitsRad, coarseResRad, fineResRad, ax)
    positions = [];
    for theta1 = jointLimitsRad(1,1):coarseResRad:jointLimitsRad(1,2)
        for theta2 = jointLimitsRad(2,1):fineResRad:jointLimitsRad(2,2)
            thetas = [theta1, theta2, 0, 0, 0, 0];
            T = transMax(0, 7, createDHTable(linkLengths, thetas));
            pos = T(1:3, 4)';
            positions(end+1, :) = pos;
        end
    end
    h = scatter3(ax, positions(:,1), positions(:,2), positions(:,3), ...
                 10, 'r', 'filled', 'DisplayName', 'Combo A');
end

%% === Combo B: Joints 1–3 Free ===
function h = plotComboB(linkLengths, jointLimitsRad, coarseResRad, fineResRad, ax)
    positions = [];
    for theta1 = jointLimitsRad(1,1):coarseResRad:jointLimitsRad(1,2)
        for theta2 = jointLimitsRad(2,1):coarseResRad:jointLimitsRad(2,2)
            for theta3 = jointLimitsRad(3,1):coarseResRad:jointLimitsRad(3,2)
                thetas = [theta1, theta2, theta3, 0, 0, 0];
                T = transMax(0, 7, createDHTable(linkLengths, thetas));
                pos = T(1:3, 4)';
                positions(end+1, :) = pos;
            end
        end
    end
    h = scatter3(ax, positions(:,1), positions(:,2), positions(:,3), ...
                 10, 'g', 'filled', 'DisplayName', 'Combo B');
end

%% === Combo C: Joints 4–6 Free (Wrist Sphere) ===
function h_all = plotComboC(linkLengths, jointLimitsRad, coarseResRad, fineResRad, ax, parentFig)
    h_all = {};
    colors = lines(64); 
    colorIdx = 1;

    t2_vals = jointLimitsRad(2,1):coarseResRad:jointLimitsRad(2,2);
    t3_vals = jointLimitsRad(3,1):coarseResRad:jointLimitsRad(3,2);

    togglePanel = uipanel('Title','Combo C Layers','FontSize',10,...
                          'Position',[0.65 0.05 0.33 0.9],...
                          'Parent', parentFig);

    % Store group checkboxes
    groupCheckboxes = cell(length(t2_vals), 1);
    theta3Checkboxes = cell(length(t2_vals), length(t3_vals));

    for i = 1:length(t2_vals)
        theta2 = t2_vals(i);

        layerPanel = uipanel('Parent', togglePanel, ...
                             'Title', sprintf('Theta2 = %d°', round(rad2deg(theta2))), ...
                             'Position', [0.05, 1 - i*0.11, 0.9, 0.1]);

        % Master checkbox for this theta2 group
        groupCheckboxes{i} = uicontrol('Style', 'checkbox', 'String', 'Toggle All T3', ...
            'Parent', layerPanel, 'Units','normalized', ...
            'Position', [0.01, 0.8, 0.3, 0.2], 'Value', 1);

        for j = 1:length(t3_vals)
            theta3 = t3_vals(j);
            positions = [];

            for theta1 = jointLimitsRad(1,1):coarseResRad:jointLimitsRad(1,2)
                baseThetas = [theta1, theta2, theta3, 0, 0, 0];

                for theta4 = jointLimitsRad(4,1):fineResRad:jointLimitsRad(4,2)
                    for theta5 = jointLimitsRad(5,1):fineResRad:jointLimitsRad(5,2)
                        thetas = baseThetas;
                        thetas(4:6) = [theta4, theta5, 0];
                        T = transMax(0, 7, createDHTable(linkLengths, thetas));
                        positions(end+1, :) = T(1:3, 4)';
                    end
                end
            end

            h = scatter3(ax, positions(:,1), positions(:,2), positions(:,3), ...
                         5, colors(colorIdx,:), 'filled', ...
                         'DisplayName', sprintf('T2=%d°, T3=%d°', ...
                         round(rad2deg(theta2)), round(rad2deg(theta3))));
            colorIdx = mod(colorIdx, size(colors,1)) + 1;
            h_all{i,j} = h;

            % Theta3 toggle
            theta3Checkboxes{i,j} = uicontrol('Style', 'checkbox', 'String', ...
                sprintf('T3 = %d°', round(rad2deg(theta3))), ...
                'Parent', layerPanel, 'Units','normalized', ...
                'Position', [mod(j-1,4)*0.25, 0.4 - floor((j-1)/4)*0.4, 0.25, 0.4], ...
                'Value', 1, ...
                'Callback', @(src,~) toggleVis(h_all{i,j}, src.Value));

        end

        % Callback to control all T3 checkboxes in the group
        groupCheckboxes{i}.Callback = @(src,~) toggleGroup(src.Value, theta3Checkboxes(i,:), h_all(i,:));
    end
end

function toggleGroup(state, checkboxList, handleList)
    for k = 1:length(checkboxList)
        checkboxList{k}.Value = state;
        toggleVis(handleList{k}, state);
    end
end

function toggleVis(h, state)
    if isgraphics(h)
        if state
            h.Visible = 'on';
        else
            h.Visible = 'off';
        end
    end
end
