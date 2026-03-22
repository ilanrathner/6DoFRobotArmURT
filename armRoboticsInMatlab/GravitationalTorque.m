%find the torque on each motor due to gravity. This is not a universal
%function because it very much depends on the masses being using and where
%they are which is variable for each usecase.
function GTorque = GravitationalTorque(DH_table)
    %masses of links. Don't need link 1 cause thats the base
    syms mL1 mL2 mL3 mL4 mL5 
    %masses of motors that will affect the torque (since parallel linkage,
    %the first three motors won't affect the torque requirements. Mass of
    %chess piece
    syms M4 M5 M6 MC

%     mL1 = 14.16 ; mL2 = 84.96; mL3 = 56.64; mL4 = 19.824; mL5 = 14.16;
%     M4 = 55; M5=34.8; M6=34.8; MC = 3;
%     mL1 = .01416 ; mL2 = .08496; mL3 = .05664; mL4 = .019824; mL5 = .01416;
%     M4 = .055; M5=.0348; M6=.0348; MC = .003;
    masses = [mL1, mL2, mL3, mL4, mL5, M4, M5, M6, MC];
    g_vec = [0; 0; -9.81];

    % Define which joint each mass is associated with
    % Index corresponds to masses = [mL1, mL2, mL3, mL4, mL5, M4, M5, M6, MC]
    % Meaning: mL1 is affected by joint 1, M4 by joint 4, MC also by joint 5, etc.
    mass_to_joint = [1, 2, 3, 4, 5, 4, 5, 5, 5];

    % Number of joints 
    num_joints = size(DH_table, 1) - 1;

    % store midpoint between each joint axis. from the definintion the
    % midpoint between axis 1 and 2 is the same place as they are on top of
    % each other. As in no link between them, so can ignore second column
    % in the midpoints below
    midpoints = sym(zeros(3, num_joints + 1));

    % Initialize position vector storage. will store positions from 0 to
    % each joint.
    P = sym(zeros(3, num_joints+1));

    %caluculate the z_i. Note the last z will not be used as thats the end
    %effector z and not related to any joints.
    z = sym(zeros(3, num_joints + 1));

    pLast = [0;0;0];
    % Compute midpoints between frame i and i+1
    for i = 1:num_joints + 1
        T_i = transMax(0, i, DH_table);
        p_i   = T_i(1:3, 4);     % origin of frame i

        P(:,i) = T_i(1:3,4);  % Store position of each joint

        z(:,i) = T_i(1:3,3);

        midpoints(:,i) = ((p_i + pLast) / 2);
        pLast = p_i;
    end   
    P_end = P(:, end); % Position of end-effector

    %create array of mass positions that correspond to the mass_to_joint
    %array. Note skipping to midpoint 3 due to earlier comment. Assume end
    %effector motor is also at the halfway point between joint 5 and EE
    mass_pos = [midpoints(:,1), midpoints(:,3), midpoints(:,4), midpoints(:,5), ...
        midpoints(:,6), P(:,4), P(:,5), midpoints(:,6), P_end];

    
    %initilise torque as zero
    GTorque = sym(zeros(num_joints, 1));

    %compute jacobians for each mass
    for j = 1:length(masses)
        %initilise empty jacobian
        JvM = sym(zeros(3, num_joints));
        joint_affects_mass = mass_to_joint(j);
        mass = masses(j)
    
        for i = 1:num_joints
            if i <= joint_affects_mass
                JvM(:, i) = cross(z(:, i), mass_pos(:, j) - P(:, i));
            end
        end

        JvM_i = simplify(JvM)

        % Add gravity torque contribution. transpose(J)*m*g_vec
        GTorque = GTorque + simplify(transpose(JvM) * masses(j) * g_vec);
    end
    GTorque = simplify(GTorque);
end
