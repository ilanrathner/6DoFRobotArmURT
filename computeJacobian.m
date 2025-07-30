%finding the jacobian matrix assuming revolute joints
function J = computeJacobian(DH_table)
    % Number of joints
    num_joints = size(DH_table, 1) - 1;

    endEffectorDHRow = num_joints + 1;
       
    % Initialize Jacobian matrix
    J = (zeros(6, num_joints));

    %compute end effector position
    T = transMax(0, endEffectorDHRow, DH_table);  % Compute cumulative transformation
    P_end = T(1:3,4);  % Store position of each joint
    
    % Compute Jacobian columns
    for i = 1:num_joints 
        % Compute cumulative transformation again
        T = transMax(0, i, DH_table);

        %position of joint
        P = T(1:3,4);
        
        % Extract Z axis of the i-th frame
        Z_i = T(1:3,3);
        
        % Compute linear velocity Jacobian. Position from i to end effector
        % in frame 0.
        J(1:3, i) = cross(Z_i, (P_end - P));
        
        % Compute angular velocity Jacobian
        J(4:6, i) = Z_i;
    end
    
    % Simplify Jacobian matrix
    %J = simplify(J);
    %J(abs(J) < tolerance) = 0;  % Set small values to zero
end
