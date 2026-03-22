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
