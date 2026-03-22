clc; clear; close all;

[linkLengths, thetas] = initializeDHVariables(5,30,20,7,5,0,0,0,0,0);

DH_table = createDHTable(linkLengths,thetas);

% initial end effector position
P_trans=transMax(0, 6, DH_table);
P_end=P_trans(1:3,4);

J= computeJacobian(DH_table);

ValidateJacobianMatrix(P_end, thetas,linkLengths);


function [linkLengths, thetas] = initializeDHVariables(L1, L2, L3, L4, E,...
                                    theta1, theta2, theta3, theta4, theta5)
    linkLengths = [L1, L2, L3, L4, E];
    thetas = [theta1, theta2, theta3, theta4, theta5]*pi/180;
end

function dh_table = createDHTable(linkLengths, thetas)
    theta1 = thetas(1); theta2 = thetas(2); theta3 = thetas(3); 
    theta4 = thetas(4); theta5 = thetas(5);
    L1 = linkLengths(1); L2 = linkLengths(2); L3 = linkLengths(3);
    L4 = linkLengths(4); E = linkLengths(5);
    
    dh_table = [0 0 L1 theta1;
                0 -pi/2 0 (theta2 - pi/2);
                L2 0 0 theta3;
                L3 0 0 (theta4 + pi/2);
                0 pi/2 L4 theta5;
                0 0 E 0];
end

