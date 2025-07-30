%calculate final transformation matix from frame j to i
function transformationMatrix = transMax(j, i, DH)
    [r,c] = size(DH);
    if i > r || j > r || c > 4 || j < 0 || i < j
        error('invalid DH table. Check the size');
    end
    transformationMatrix = eye(4);
    if j<i
        %j+1 because input can be zero but this is for row 1
        for F = (j+1):i
            transformationMatrix = transformationMatrix * transform(DH(F, :));
        end
     end
     %transformationMatrix = simplify(transformationMatrix);
end

%transformation matrix for a given row of dhTable/frame
function T = transform(dh_row)
    theta = dh_row(4);  % Rotation about z-axis
    alpha = dh_row(2);  % Rotation about x-axis
    a     = dh_row(1);  % Translation along x-axis
    d     = dh_row(3);  % Translation along z-axis

    % Correct DH transformation matrix order: Tx(a) * Rx(alpha) * Tz(d) * Rz(theta)
    T = translate(a, 0, 0) * rotMax('x', alpha) * translate(0, 0, d) * rotMax('z', theta);
end

%find translation matrix given translation along x,y,z
function translationalMatrix = translate(x,y,z)
    translationalMatrix = [1 0 0 x;
                           0 1 0 y;
                           0 0 1 z;
                           0 0 0 1];
end

%find rotation matrix given the axis of rotation and angle in radians
function rotationMatrix = rotMax(type, angle)
    if type == 'x'
        rotationMatrix = [1 0 0 0; 
                     0 cos(angle) -sin(angle) 0; 
                     0 sin(angle) cos(angle) 0;
                     0 0 0 1];
    elseif type == 'y'
        rotationMatrix = [cos(angle) 0 sin(angle) 0; 
                     0 1 0 0; 
                     -sin(angle) 0 cos(angle) 0;
                     0 0 0 1];
    elseif type == 'z'
        rotationMatrix = [cos(angle) -sin(angle) 0 0; 
                     sin(angle) cos(angle) 0 0; 
                     0 0 1 0;
                     0 0 0 1];
    else 
        error("invalid type");
    end
end