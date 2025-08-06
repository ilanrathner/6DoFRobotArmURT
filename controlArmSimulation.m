%function that will update an arm simulation when link lengths, thetas,
%thetas work in radians in the background in other functions but since its
%simpler to visualize the input field here will be in degrees.
%linklengths are in cm
%position or orientation is changed
function [linkLengths, thetas] = controlArmSimulation(linkLengths, thetas, arm)
    fig = uifigure('Name', 'Update DH Parameters', 'Position', [100, 100, 950, 550]);

    % Create input fields for link lengths
    lFields = gobjects(1, length(linkLengths));
    for i = 1:length(linkLengths)
        uilabel(fig, 'Text', ['L', num2str(i)], 'Position', [50, 500 - (i * 40), 50, 20]);
        lFields(i) = uieditfield(fig, 'numeric', ...
            'Position', [100, 500 - (i * 40), 100, 22], ...
            'Value', linkLengths(i), ...
            'ValueChangedFcn', @(fld, event) onLinkLengthChanged());
    end

    % Create input fields for thetas
    thetaFields = gobjects(1, length(thetas));
    for i = 1:length(thetas)
        uilabel(fig, 'Text', ['\theta', num2str(i)], 'Position', [250, 500 - (i * 40), 50, 20]);
        thetaFields(i) = uieditfield(fig, 'numeric', ...
            'Position', [300, 500 - (i * 40), 100, 22], ...
            'Limits', [-180, 180], ...
            'Value', (180/pi)*thetas(i), ...
            'ValueChangedFcn', @(fld, event) onThetaChanged());
    end

    % Create input fields for end-effector position and orientation
    labels = {'X', 'Y', 'Z', 'Pitch', 'Yaw', 'Roll'};
    positionFields = gobjects(1, 6);
    for i = 1:6
        uilabel(fig, 'Text', labels{i}, 'Position', [450, 500 - (i * 40), 50, 20]);
        positionFields(i) = uieditfield(fig, 'numeric', ...
            'Position', [500, 500 - (i * 40), 100, 22], ...
            'Value', 0, ...
            'ValueChangedFcn', @(fld, event) onPositionChanged());
    end

    % Status label
    statusLabel = uilabel(fig, 'Text', '', 'Position', [50, 40, 800, 30], 'FontSize', 12);

    % --- Main update functions ---
    function onLinkLengthChanged()
        for i = 1:length(linkLengths)
            linkLengths(i) = lFields(i).Value;
        end
        updateForward(); % Recalculate pose
    end

    function onThetaChanged()
        for i = 1:length(thetas)
            thetas(i) = (pi/180)*thetaFields(i).Value;
        end
        updateForward(); % Recalculate pose
    end

    function onPositionChanged()
        xyzrpy = arrayfun(@(f) f.Value, positionFields);
        x = xyzrpy(1); y = xyzrpy(2); z = xyzrpy(3);

        % Convert pitch, yaw, roll from degrees to radians
        pitch = (pi/180) * xyzrpy(4);  %% <--
        yaw   = (pi/180) * xyzrpy(5);  %% <--
        roll  = (pi/180) * xyzrpy(6);  %% <--

        try
            [theta1, theta2, theta3, theta4, theta5, theta6] = inverseKinematics( ...
                x, y, z, yaw, pitch, roll, linkLengths);

            thetas = [theta1, theta2, theta3, theta4, theta5, theta6];
            for i = 1:length(thetas)
                thetaFields(i).Value = (180/pi)*thetas(i);
            end

            updateForward();
        catch
            statusLabel.Text = '❌ Target position/orientation not reachable with current link lengths.';
        end
    end

    function updateForward()
        % Update DH table and transformation
        dh_table = createDHTable(linkLengths, thetas);
        T = transMax(0, 7, dh_table); % 4x4 transformation matrix

        pos = T(1:3, 4);
        R = T(1:3, 1:3);
        [yaw, pitch, roll] = inverseOrientation(R);

        % Update GUI fields
        positionFields(1).Value = pos(1);
        positionFields(2).Value = pos(2);
        positionFields(3).Value = pos(3);
        %converting to degrees for display. When calculating inverse
        %kinematics it is converted back to radians
        positionFields(4).Value = (180/pi)*pitch;  %% <--
        positionFields(5).Value = (180/pi)*yaw;    %% <--
        positionFields(6).Value = (180/pi)*roll;   %% <--

        % Update arm display
        updateArm(arm, dh_table);

        % Update status
        statusLabel.Text = sprintf('✅ Updated: Link Lengths = [%s], Thetas = [%s]', ...
            num2str(linkLengths), num2str(thetas));
    end

    % Initialize display
    updateForward();

    uiwait(fig);
end
