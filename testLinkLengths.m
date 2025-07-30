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

