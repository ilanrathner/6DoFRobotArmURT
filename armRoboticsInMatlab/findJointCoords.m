function armJointCoords = findJointCoords(dh_table)
    zero = [0;0;0;1];

    F1 = transMax(0,1,dh_table) * zero;
    F2 = transMax(0,2,dh_table) * zero;
    F3 = transMax(0,3,dh_table) * zero;
    F4 = transMax(0,4,dh_table) * zero;
    F5 = transMax(0,5,dh_table) * zero;
    F6 = transMax(0,6,dh_table) * zero;
    F7 = transMax(0,7,dh_table) * zero;
    
    % Extract (x, y, z) coordinates for plotting
    X = [zero(1), F1(1), F2(1), F3(1), F4(1), F5(1), F6(1), F7(1)];
    Y = [zero(2), F1(2), F2(2), F3(2), F4(2), F5(2), F6(2), F7(2)];
    Z = [zero(3), F1(3), F2(3), F3(3), F4(3), F5(3), F6(3), F7(3)];

    armJointCoords = [X; Y; Z];
end