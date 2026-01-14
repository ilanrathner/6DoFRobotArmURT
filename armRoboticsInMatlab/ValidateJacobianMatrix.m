%Validate Jacobian matrix
function [P_end_new_FK, P_end_new_Jacob]= ValidateJacobianMatrix(P_end, thetas, linkLengths)

    % define joint velocity. change in degrees per second
    Q_Velocity=5*[1,1,1,1,1];

    % adjust the joint angle
    thetas = thetas+ Q_Velocity;

    % calculate new DH table
    DH_table_new = createDHTable(linkLengths,thetas);

    % do the forward kinematics
    forwardTransform = transMax(0,6,DH_table_new);
    P_end_new_FK=forwardTransform (1:3,4);

    % calculate new Jacobian
    J= computeJacobian(DH_table_new);
    
    % calculate new x,y,z velocity using jacobian matrix 
    % Displacement = velocity * time step (1 second in this case)
    Displacement=J*transpose(Q_Velocity);
    P_end_new_Jacob=P_end+Displacement(1:3);

end