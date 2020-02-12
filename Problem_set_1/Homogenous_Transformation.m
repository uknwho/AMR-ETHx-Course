alpha = sym('alpha','real');
beta = sym('beta','real');
gamma = sym('gamma','real');

% rotational matrices calculated in previous problem set
R_B1 = [1,0,0;0,cos(alpha),-sin(alpha);0,sin(alpha),cos(alpha)];
R_12 = [cos(beta),0,sin(beta);0,1,0;-sin(beta),0,cos(beta)];
R_23 = [cos(gamma),0,+sin(gamma);0,1,0;-sin(gamma),0,cos(gamma)];
x=R_B1*R_12;
y=x*R_23;
% write down the 3x1 relative position vectors for link length l_i=1
r_B1_inB = [0;1;0];
r_12_in1 = [0; 0;-1];
r_23_in2 = [0;0;-1];
r_3F_in3 = [0;0;-1]; 
disp(r_B1_inB);
disp(r_12_in1);
disp(r_3F_in3);
disp(y);
r1=[0;1;-2];

% write down the homogeneous transformation matrices
H_B1 = [1,0,0,0;0,cos(alpha),-sin(alpha),1;0,sin(alpha),cos(alpha),0;0,0,0,1];
H_12 = [cos(beta),0,sin(beta),0;0,1,0,0;-sin(beta),0,cos(beta),-1;0,0,0,1];
H_23 = [cos(gamma),0,+sin(gamma),0;0,1,0,0;-sin(gamma),0,cos(gamma),-1;0,0,0,1];
disp(H_B1);
disp(H_12);

% create the cumulative transformation matrix
H_B3 = H_B1*H_12*H_23;
% find the foot point position vector
r_BF_inB =r_B1_inB + R_B1*r_12_in1 + R_B1*R_12*r_23_in2 + R_B1*R_12*R_23*r_3F_in3;
     