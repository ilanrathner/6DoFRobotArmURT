%find the joint angles given the x,y,z coordinates and the angle phi of the
%end effector from the zero position. Also give linklengths.
function [theta1, theta2, theta3, theta4, theta5, theta6] = inverseKinematics(x,y,z,...
                                                     yaw, pitch, roll, linkLengths)
    L1 = linkLengths(1); L2 = linkLengths(2); L3 = linkLengths(3);
    L4 = linkLengths(4); L5 = linkLengths(5);

    %step 1 find the orientation rotation matirx with the pitch yaw and
    %roll
    R = OrientationMat(yaw, pitch, roll);

    %step 2 find w
    d = (L4 + L5);
    wx = x - d*R(1,3);
    wy = y - d*R(2,3);
    wz = z - d*R(3,3);

    %step 3 calculate theta 1
    theta1 = atan2(wy,wx);

    %step 4 create variables r and s for calculation of theta 2 and 3
    r = sqrt(wx^2 + wy^2);
    s = wz - L1;

    %step 5 calculate theta 3 
    theta3 = acos((r^2 + s^2 - L2^2 - L3^2)/(2 * L2 * L3));

    %step 6 calculate theta 2 now that we have theta 3
    theta2 = atan2(r, s) - asin((L3 * sin(theta3))/(sqrt(r^2 + s^2)));

    %inbetween step do calculations that are going to be repeated over and
    %over as a pre-step
    c1 = cos(theta1);
    c23 = cos(theta2+theta3);
    s1 = sin(theta1);
    s23 = sin(theta2+theta3);

    %step 7 calculate theta 4
    theta4 = atan2(R(2,3) * c1 - R(1,3)*s1, ...
        R(1,3)*c23*c1 - R(3,3)*s23 + R(2,3)*c23*s1);

    %step 8 calculate theta 5
    theta5 = atan2(sqrt(1 - (-R(3,3)*c23 - R(1,3)*s23*c1 - R(2,3)*s23*s1)^2), ...
        -(-R(3,3)*c23 - R(1,3)*s23*c1 - R(2,3)*s23*s1));

    %step 9 calculate theta 6
    theta6 = atan2(-(-R(3,2)*c23 - R(1,2)*s23*c1 - R(2,2)*s23*s1), ...
        -R(3,1)*c23 - R(1,1)*s23*c1 - R(2,1)*s23*s1);

    % Check if any of the calculated angles have imaginary components
    if ~isreal([theta1, theta2, theta3, theta4, theta5, theta6])
        error('One or more joint angles are complex. Target position is not in the reachable workspace.');
    end
end

%yaw is rotation about z, pitch is rotation about y, roll is rotation about
%x. Here it is all anti-clockwise rotation
function OrientationMatrix = OrientationMat(yaw, pitch, roll)
    XrotationMatrix = [1 0 0; 
                       0 cos(roll) -sin(roll); 
                       0 sin(roll) cos(roll)];

    YrotationMatrix = [cos(pitch) 0 sin(pitch); 
                       0 1 0; 
                       -sin(pitch) 0 cos(pitch)];

    ZrotationMatrix = [cos(yaw) -sin(yaw) 0; 
                       sin(yaw) cos(yaw) 0; 
                       0 0 1];

    OrientationMatrix = ZrotationMatrix*YrotationMatrix*XrotationMatrix;
    
end
