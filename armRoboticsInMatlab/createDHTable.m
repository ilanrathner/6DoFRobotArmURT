function dh_table = createDHTable(linkLengths, thetas)
    theta1 = thetas(1); theta2 = thetas(2); theta3 = thetas(3); 
    theta4 = thetas(4); theta5 = thetas(5); theta6 = thetas(6);
    L1 = linkLengths(1); L2 = linkLengths(2); L3 = linkLengths(3);
    L4 = linkLengths(4); L5 = linkLengths(5);

    pi2 = pi/2;

    dh_table = [0 0 L1 theta1;
                0 -pi2 0 (theta2-pi2);
                L2 0 0 (theta3+pi2);
                0 pi2 L3 theta4;
                0 -pi2 0 theta5;
                0 pi2 L4 theta6;
                0 0 L5 0];
    
end