function [yaw, pitch, roll] = inverseOrientation(OrientationRotationMatrix)
    R = OrientationRotationMatrix;
    pitch = asin(-R(3,1));
    roll = atan2(R(3,2),R(3,3));
    yaw = atan2(R(2,1),R(1,1));
end