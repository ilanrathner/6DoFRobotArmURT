clear
close all
clc

%% Transformation Matrices

syms theta1 theta2 theta3 theta4 theta5 L1 L2 L3 L4 E

dh_table = [0 0 L1 theta1;
               0 -90 0 (theta2 - 90);
               L2 0 0 theta3;
               L3 0 0 (theta4 + 90);
               0 90 L4 theta5;
               0 0 E 0];

T01 = transMax(0,1,dh_table);
T02 = transMax(0,2,dh_table);
T03 = transMax(0,3,dh_table);
T04 = transMax(0,4,dh_table);
T05 = transMax(0,5,dh_table);
T0E = transMax(0,6,dh_table);

%% Jacobian differentiation, this only works for the linear velocities, this does not work for the angular velocities
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);

P0_01 = T01(1:3,4);
P0_02 = T02(1:3,4);
P0_03 = T03(1:3,4);
P0_04 = T04(1:3,4);
P0_05 = T05(1:3,4);
P0_0E = T0E(1:3,4);

P0_1E = P0_0E - P0_01;
P0_2E = P0_0E - P0_02;
P0_3E = P0_0E - P0_03;
P0_4E = P0_0E - P0_04;
P0_5E = P0_0E - P0_05;

dJ = [simplify(diff(T0E(1:3,4),theta1)) simplify(diff(T0E(1:3,4),theta2))...
    simplify(diff(T0E(1:3,4),theta3)) simplify(diff(T0E(1:3,4),theta4))...
    simplify(diff(T0E(1:3,4),theta5))]

J= computeJacobian(dh_table);
J(1:3,:)

%%
subdJ = double(rad2deg(subs(dJ,[theta1 theta2 theta3 theta4 theta5 L1 L2 L3 L4 E],[30 30 30 30 30 5 5 5 5 5])))
subJ = double(subs(J,[theta1 theta2 theta3 theta4 theta5 L1 L2 L3 L4 E],[30 30 30 30 30 5 5 5 5 5]))


function iterativeLinkLengthFinder()
    theta1_range = linspace(20, 160, 15);  % Base rotation (Yaw) for XY plane
    theta1_range_YZ = linspace(90, 90, 1);  % Base rotation (Yaw) for YZ plane
    theta2_range = linspace(20, 160, 15);  % Shoulder (Pitch)
    theta3_range = linspace(20, 160, 15);  % Elbow (Pitch)
    theta4_range = linspace(20, 160, 15);  % Wrist 1 (Roll)
    theta5_range = linspace(90, 90, 1);  % Wrist 2 (Pitch)
    
    % plot the chessboard for XY plane
    board = initializeBoard(35, [-17,5,0]);
   
    % loop to plot every possible arm reach point on the XY plane for the given 
    % operation angle range
    for theta1 = theta1_range
        for theta2 = theta2_range
            for theta3 = theta3_range
                for theta4 = theta4_range
                    for theta5 = theta5_range
                        iterativeLLF_HelperFunc(theta1,theta2,theta3,theta4,theta5,true)
                    end
                end
            end
        end
    end

    % plot the chessboard for YZ plane
    board = initializeBoard(35, [-17,5,0]);

    % loop to plot every possible arm reach point on the YZ plane for the given 
    % operation angle range
    for theta1 = theta1_range_YZ
        for theta2 = theta2_range
            for theta3 = theta3_range
                for theta4 = theta4_range
                    for theta5 = theta5_range
                        iterativeLLF_HelperFunc(theta1,theta2,theta3,theta4,theta5,false)
                    end
                end
            end
        end
    end
end

function iterativeLLF_HelperFunc(theta1,theta2,theta3,theta4,theta5,flag)
    % the parameters below are linka lengths, which can be adjust to see
    % the change in arm reach space
    L1=5;
    L2=30;
    L3=20;
    L4= 7;
    E=5;

    DH_table = [0 0 L1 theta1;
                0 -90 0 (theta2 - 90);
                L2 0 0 theta3;
                L3 0 0 (theta4 + 90);
                0 90 L4 theta5;
                0 0 E 0];   
    
    zero = [0;0;0;1];
    
    F1 = transMax(0,1,DH_table) * zero;
    F2 = transMax(0,2,DH_table) * zero;
    F3 = transMax(0,3,DH_table) * zero;
    F4 = transMax(0,4,DH_table) * zero;
    F5 = transMax(0,5,DH_table) * zero;
    F6 = transMax(0,6,DH_table) * zero;
    
    if F1(3)<0 || F2(3)<0 || F3(3)<0 || F4(3)<0 || F5(3)<0 || F6(3)<0
        return
    end
    
    if F6(2)< 5
        return
    end
    
    % if flag is true, then it is ploting XY plane
    if flag
        if F6(3)>3.5
            return
        end
         
        if F6(3)<2.5
            return
        end
    end

    X = [F6(1)];
    Y = [F6(2)];
    Z = [F6(3)];    
    
    % Plot the stick figure
    plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    view(3); % 3D View
    hold on;
end

%function that uses the inverse kinematics to test whether each position is
%reachable given the found link lengths
function verifiedLinkLengths = testLinkLengths(linkLengths, sideLength, startPoint)
    % Determine square size based on total side length (8x8 board)
    numberSquares = 8;  % Standard chessboard
    squareSize = sideLength / numberSquares;

    x_st = startPoint(1) / squareSize;
    y_st = startPoint(2) / squareSize;
    z_st = startPoint(3) ;

    for x = x_st:(x_st + numberSquares - 1)
        x_coord = x*squareSize + squareSize/2;
        %going to every square on the board
        for y = y_st:(y_st + numberSquares - 1)
            y_coord = y*squareSize + squareSize/2;
            %magic numbers are the pieces hieghts
            for z = [z_st + 2, z_st + 5, z_st + 15]
                z_coord = z;
                disp([x_coord,y_coord,z_coord]);
                [theta1, theta2, theta3, theta4, theta5] = ...
                    inverseKinematics(x_coord,y_coord,z_coord, 180, linkLengths);
                thetas = [theta1, theta2, theta3, theta4, theta5];
                DH_table = createDHTable(linkLengths,thetas);
                arm = initialiseArm(DH_table);
                if ~isreal([theta1, theta2, theta3, theta4, theta5])
                    verifiedLinkLengths = false;
                    return
                end
            end
        end
    end
    verifiedLinkLengths = true;
end



%function used to find the link lengths that will allow every position to
%be reached
function [linkLengths, thetas] = controlArmWithSliders(linkLengths, thetas,arm)
    % Create a UI figure
    fig = uifigure('Name', 'Update DH Parameters', 'Position', [100, 100, 800, 400]);

    % Create sliders for link lengths
    lSliders = gobjects(1, length(linkLengths));
    for i = 1:length(linkLengths)
        uilabel(fig, 'Text', ['L', num2str(i)], 'Position', [50, 350 - (i * 50), 50, 20]);
        lSliders(i) = uislider(fig, ...
            'Position', [100, 350 - (i * 50), 200, 3], ...
            'Limits', [1, 50], ...
            'Value', linkLengths(i), ...
            'ValueChangedFcn', @(sld, event) updateValues());
    end

     % Create sliders for thetas
     thetaSliders = gobjects(1, length(thetas));
     for i = 1:length(thetas)
         uilabel(fig, 'Text', ['\theta', num2str(i)], 'Position', [400, 350 - (i * 50), 50, 20]);
         thetaSliders(i) = uislider(fig, ...
             'Position', [400, 350 - (i * 50), 200, 3], ...
             'Limits', [-180, 180], ...
             'Value', thetas(i), ...
             'ValueChangedFcn', @(sld, event) updateValues());
     end

    % Display updated values
    outputLabel = uilabel(fig, 'Text', '', 'Position', [100, 50, 300, 50], 'FontSize', 12);

    function updateValues()
        for i = 1:length(linkLengths)
            linkLengths(i) = lSliders(i).Value;
        end
        for i = 1:length(thetas)
            thetas(i) = thetaSliders(i).Value;
        end
        outputLabel.Text = sprintf('Link Lengths: [%s]\nThetas: [%s]', ...
            num2str(linkLengths), num2str(thetas));
        DH_table = createDHTable(linkLengths,thetas);
        updateArm(arm,DH_table);
    end

    % Wait for the user to close the GUI before returning values
    uiwait(fig);
end

function armJointCoords = findJointCoords(dh_table)
    zero = [0;0;0;1];

    F1 = transMax(0,1,dh_table) * zero;
    F2 = transMax(0,2,dh_table) * zero;
    F3 = transMax(0,3,dh_table) * zero;
    F4 = transMax(0,4,dh_table) * zero;
    F5 = transMax(0,5,dh_table) * zero;
    F6 = transMax(0,6,dh_table) * zero;
    
    % Extract (x, y, z) coordinates for plotting
    X = [zero(1), F1(1), F2(1), F3(1), F4(1), F5(1), F6(1)];
    Y = [zero(2), F1(2), F2(2), F3(2), F4(2), F5(2), F6(2)];
    Z = [zero(3), F1(3), F2(3), F3(3), F4(3), F5(3), F6(3)];

    armJointCoords = [X; Y; Z];
end

function arm = initialiseArm(dh_table)
    coords = findJointCoords(dh_table);
    X = coords(1,:);
    Y = coords(2,:);
    Z = coords(3,:);
    % Plot the stick figure
    arm = plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');    
end

function updateArm(armPlot, dh_table)
    coords = findJointCoords(dh_table);
    X = coords(1,:);
    Y = coords(2,:);
    Z = coords(3,:);
    %update the plot of the arm with new coordinates
    set(armPlot, 'XData',X, 'YData', Y, 'ZData', Z);

     % Pause to allow visualization
     pause(0.1);
end

%funtion to initialize the position of the pieces. All pieces are cylinders
%for simplicity and visualization
function [pieces, piecePositions] = initializePieces(sideLength, startPoint)
    %piece information
    numOfPieces = 16;
    pieceHeight = 3;
    baseRadius = 1;

    % Determine square size based on total side length (8x8 board)
    numberSquares = 8;  % Standard chessboard
    squareSize = sideLength / numberSquares;

    %/square size cause lower multiplying by
    x_st = startPoint(1) / squareSize;
    y_st = startPoint(2) / squareSize;
    z_st = startPoint(3) ;

    %storing so that they can be modified (moved)
    pieces = gobjects(numOfPieces,1);
    piecePositions = zeros(numOfPieces, 2);

    %pieces starting positions. -1 so that only 8 x points are given
    %starting from the corner of the board(the startPoint). dividing by
    %numberSquares earlier and multiplying now so that we only make a piece
    %for each square. two inner for loops. one for white other for black
    piece = 1;
    for x = x_st:(x_st+numberSquares - 1)
        x_coord = x*squareSize + squareSize/2;
        %only need the start position and one square above it for 2 rows of
        %pieces in chess
        for y = y_st:(y_st+1)
            y_coord = y*squareSize + squareSize/2;
            piecePositions(piece,1) = x_coord;
            piecePositions(piece,2) = y_coord;
            
            %create piece plot
            [X, Y, Z] = cylinder(baseRadius, 30);
            Z = Z * pieceHeight;
            X = X + x_coord;
            Y = Y + y_coord;

            % Plot the cylinder and store its handle
            pieces(piece) = surf(X, Y, Z, 'FaceColor', "white", 'EdgeColor', 'red');
            view(3);

            piece = piece + 1;
        end
        %giving y position for black rows. This is from the third last line
        %to the second last. The last line is the border
        for y = (y_st + numberSquares - 2):(y_st + numberSquares - 1)
            y_coord = y*squareSize + squareSize/2;
            piecePositions(piece,1) = x_coord;
            piecePositions(piece,2) = y_coord;

            %create piece plot
            [X, Y, Z] = cylinder(baseRadius, 30);
            Z = Z * pieceHeight;
            X = X + x_coord;
            Y = Y + y_coord;

            % Plot the cylinder and store its handle
            pieces(piece) = surf(X, Y, Z, 'FaceColor', "black", 'EdgeColor', 'black');
            view(3);

            piece = piece + 1;
        end
    end

    %create piece graphs and store in pices object
end

%function to move a piece given its index and new position
function movePiece(pieceIndex, pieces, positions, newX, newY)
    pieceHeight = 3;
    baseRadius = 1;
    % Get current position
    oldX = positions(pieceIndex, 1);
    oldY = positions(pieceIndex, 2);

    % Update position
    positions(pieceIndex, :) = [newX, newY];

    % Move the piece visually
    if isvalid(pieces(pieceIndex))
        [X, Y, Z] = cylinder(baseRadius, 30);
        Z = Z * pieceHeight;
        X = X + newX;
        Y = Y + newY;
        set(pieces(pieceIndex), 'XData', X, 'YData', Y, 'ZData', Z);
    end
end

%create a chess board with assumtion that squares are uniform so sidelength
%can be divided by 8 to make the squares. StartPoint is an array [x,y,z]
function board = initializeBoard(sideLength, startPoint)
    % Determine square size based on total side length (8x8 board)
    numberSquares = 8;  % Standard chessboard
    squareSize = sideLength / numberSquares;

     % Start points for positioning
    x_st = startPoint(1);
    y_st = startPoint(2);
    z_st = startPoint(3);

    % Create figure
    figure;
    hold on;
    axis equal;
    grid on;
    
    % Set board limits. Magic numbers are the experimental values that make
    % it look nice
    xlim([x_st - sideLength/3, x_st + sideLength + 10]);
    ylim([y_st - sideLength/3, y_st + sideLength + 10]);
    zlim([-1, 2*sideLength + z_st]);  % Slight elevation for visibility

    % Draw squares with alternating colors. starting with zero and ending
    % with nS-1 means 8 points still given starting at x_st
    for row = 0:numberSquares-1
        for col = 0:numberSquares-1
            % Compute bottom-left corner of the square
            x1 = x_st + col * squareSize;
            y1 = y_st + row * squareSize;

            % Define the 4 corner points of the square
            x = [x1, x1 + squareSize, x1 + squareSize, x1];
            y = [y1, y1, y1 + squareSize, y1 + squareSize];
            z = [0, 0, 0, 0];  % Keep squares on the XY plane at z = 0

            % Choose color: White if (row+col) is odd, else Black
            if mod(row + col, 2) == 0
                color = [0, 0, 0]; % Black
            else
                color = [1, 1, 1]; % White
            end

            % Create the square using 'patch'
            patch(x, y, z, color, 'EdgeColor', 'k', 'LineWidth', 2);
        end
    end
    
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    title('3D Chessboard');
    
    % Return board handle for future modifications
    board = gca;
end




