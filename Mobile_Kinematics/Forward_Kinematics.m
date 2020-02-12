%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 1: paste values for alpha, beta, and ell from   %
%         the answer to the last question              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wheel 1, the far right wheel
alpha1= 0;
beta1= 0;
ell1= 1/2;

% Wheel 2, the top left wheel
alpha2= 2*pi/3;
beta2= 0;
ell2= 1/2;
      
% Wheel 3, the bottom left wheel
alpha3= 4*pi/3;
beta3= 0;
ell3= 1/2;
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 2: derive the matrices J and R                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = [...
    sin(alpha1+beta1 ),-cos(alpha1+beta1),-ell1*cos(beta1);...
    sin(alpha2+beta2 ),-cos(alpha2+beta2),-ell2*cos(beta2);...
    sin(alpha3+beta3 ),-cos(alpha3+beta3),-ell3*cos(beta3)];
R = [0.1,0,0;0,0.1,0;0,0,0.1]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 3: Compute the forward differential             %
%         kinematics matrix, F                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = inv(J)*R