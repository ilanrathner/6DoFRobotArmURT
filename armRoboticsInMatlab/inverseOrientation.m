function [yaw, pitch, roll] = inverseOrientation(OrientationRotationMatrix)
    R = OrientationRotationMatrix;

    tolerance = 1e-9;

    % Apply the rounding function to each term
    R31_rounded = round_to_zero(R(3,1), tolerance);
    R32_rounded = round_to_zero(R(3,2), tolerance);
    R33_rounded = round_to_zero(R(3,3), tolerance);
    R21_rounded = round_to_zero(R(2,1), tolerance);
    R11_rounded = round_to_zero(R(1,1), tolerance);
    
    % Perform the final calculations
    pitch = asin(-R31_rounded);
    roll = atan2(R32_rounded, R33_rounded);
    yaw = atan2(R21_rounded, R11_rounded);
end

function rounded_val = round_to_zero(val, tolerance)
    if abs(val) < tolerance
        rounded_val = 0;
    else
        rounded_val = val;
    end
end

