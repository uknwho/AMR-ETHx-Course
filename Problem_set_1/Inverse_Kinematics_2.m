% given are the functions 
%   r_BF_inB(alpha,beta,gamma) and
%   J_BF_inB(alpha,beta,gamma) 
% for the foot positon respectively Jacobian

r_BF_inB = @(alpha,beta,gamma)[...
    -sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];
 
J_BF_inB = @(alpha,beta,gamma)[...
                                              0,             - cos(beta + gamma) - cos(beta),            -cos(beta + gamma);...
 cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
 sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];
 
% write an algorithm for the inverse kinematics problem to
% find the generalized coordinates q that gives the endeffector position rGoal =
% [0.2,0.5,-2]' and store it in qGoal
q0 = pi/180*([0,-30,60])';
rGoal = [-1.5,1,-2.5]';


tor = 1e-8;
% initial positions
q = q0;
r1 = rGoal
r = r_BF_inB(q(1),q(2),q(3));

% iterations, this is the singularity case.

while norm(r1-r) > tor
    r1=r;
    q = q + 0.1*(J_BF_inB(q(1),q(2),q(3))')*(rGoal-r);  % transpose is the inverse 
    r = r_BF_inB(q(1),q(2),q(3)); % updating the new end position
end

disp(rad2deg(q)) % the best possible values of angles fo which the rGoal can be achieved
qGoal = q;   