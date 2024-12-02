function [M] = A_Linearization_Dubin_Car(x,u)
M=[ 1, 0, -u(1)*sin(x(3));
   0, 1,  u(1)*cos(x(3));
   0, 0,           1];
end

