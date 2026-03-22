% Create transformation matrix from one line of the dh table
% Takes the input of a dh table row as a 1x4 matrix [a,alpha,d,theta]
% Returns a 4x4 transformation matrix
% Transformation Matrix = TransMat(dh_row)
function TransMat = TransMat(dh_row)
    a = dh_row(1);
    alpha = dh_row(2);
    d = dh_row(3);
    theta = dh_row(4);

    m1 = [1 0 0 a;...
        0 cos(alpha) -sin(alpha) 0;...
        0 sin(alpha) cos(alpha) 0;...
        0 0 0 1];

    m2 = [cos(theta) -sin(theta) 0 0;...
          sin(theta) cos(theta) 0 0;...
          0 0 1 d;...
          0 0 0 1];

    TransMat = m1*m2;


end

