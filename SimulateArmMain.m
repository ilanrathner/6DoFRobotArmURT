linklengths = [10;10;10;10;10];
thetas = [90,90,0,180,90,90] * pi/180;
dh = createDHTable(linklengths, thetas);

%T = transMax(0,7,dh);
%R = T(1:3,1:3);
%[yaw,pitch,roll] = inverseOrientation(R)

%armSim = initialiseArm(dh);

[t1,t2,t3,t4,t5,t6] = inverseKinematics(0,20,30, 0,0,0, linklengths);
ts = [t1,t2,t3,t4,t5,t6] * 180/pi

% A board facing X-axis, centered at x=15, z=10
drawVerticalBoard('x', 15, 30, 60, 90);

hold on

armSim = initialiseArm(dh);
[linklengths, thetas] = controlArmSimulation(linklengths, thetas, armSim);


function drawVerticalBoard(axisDirection, axisOffset, baseHeight, boardWidth, boardHeight)
% drawVerticalBoard Draws a vertical board in 3D space.
%
% Parameters:
%   axisDirection - 'x' or 'y' (board is centered along this axis)
%   axisOffset    - position along that axis where bottom center is placed
%   baseHeight    - z-axis offset (height of bottom-middle edge)
%   boardWidth    - width of board (extends perpendicular to axisDirection)
%   boardHeight   - height of board (along z)

    % Validate input
    if ~ismember(axisDirection, {'x', 'y'})
        error("axisDirection must be either 'x' or 'y'");
    end

    w = boardWidth / 2;
    h = boardHeight;

    % Define board corners based on direction
    switch axisDirection
        case 'x'
            % Width spans along Y, center at (x=axisOffset, y=0, z=baseHeight)
            x = [axisOffset, axisOffset, axisOffset, axisOffset];
            y = [-w, w, w, -w];
        case 'y'
            % Width spans along X, center at (x=0, y=axisOffset, z=baseHeight)
            x = [-w, w, w, -w];
            y = [axisOffset, axisOffset, axisOffset, axisOffset];
    end

    z = [baseHeight, baseHeight, baseHeight + h, baseHeight + h];

    % Close polygon
    x(end+1) = x(1);
    y(end+1) = y(1);
    z(end+1) = z(1);

    % Plot
    figure;
    plot3(x, y, z, 'b-', 'LineWidth', 2);
    fill3(x, y, z, [0.8, 0.8, 0.8]);

    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('Vertical Board (Centered on %s-axis)', upper(axisDirection)));
    axis equal;
    grid on;
    view(3);
end
