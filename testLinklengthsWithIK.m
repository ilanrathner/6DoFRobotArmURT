%this function is to test whether the arm with this configuration of linklengths 
% can reach a point. The orientation is given as input and is the pitch yaw
% roll for robustness of the function but it is expected to be the
% orientation to point the end effecter straight at the panel ie ze points
% towars panel. This is given by [0 0 1; 0 1 0;-1 0 0] which is 0 90 0 YPR
function testLinklengthsWithIK(linkLengths, orientationYPR, testPoints)
    % Create a figure for plotting
    fig = figure('Name', 'Arm Reachability Test', 'NumberTitle', 'off');
    hold on;
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Arm Poses for Reachable Points');
    
    % Set tolerance for validation
    tolerance = 1e-3;

    % Loop through each test point
    for i = 1:size(testPoints, 1)
        x = testPoints(i,1);
        y = testPoints(i,2);
        z = testPoints(i,3);
        yaw = orientationYPR(1);
        pitch = orientationYPR(2);
        roll = orientationYPR(3);

        try
            % Solve inverse kinematics
            [theta1, theta2, theta3, theta4, theta5, theta6] = inverseKinematics(x, y, z, yaw, pitch, roll, linkLengths);
            jointAngles = [theta1, theta2, theta3, theta4, theta5, theta6];

            % Check for invalid solutions
            if any(isnan(jointAngles)) || any(abs(jointAngles) > pi + tolerance)
                error('Unreachable point at index %d: [%f %f %f]', i, x, y, z);
            end

            % Create DH table and plot the arm
            dh_table = createDHTable(linkLengths, jointAngles);
            initialiseArm(dh_table);

        catch ME
            % Close figure and rethrow error
            close(fig);
            rethrow(ME);
        end
    end

    hold off;
end