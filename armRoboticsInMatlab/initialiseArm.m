function arm = initialiseArm(dh_table)
    coords = findJointCoords(dh_table);
    X = coords(1,:);
    Y = coords(2,:);
    Z = coords(3,:);
    % Plot the stick figure
    arm = plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');    
end