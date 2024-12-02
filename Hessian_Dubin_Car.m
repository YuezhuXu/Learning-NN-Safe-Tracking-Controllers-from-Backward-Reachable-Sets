function [H1,H2,H3] = Hessian_Dubin_Car(x,Rx,u,Ru)


theta_interval=interval(x(3)-Rx(3),x(3)+Rx(3));
u1_interval=abs(interval(u(1)-Ru(1),u(1)+Ru(1)));
sup_u1_interval=u1_interval.sup;
cs_theta_interval=abs(cos(theta_interval));
sup_cs_theta_interval=cs_theta_interval.sup;
sn_theta_interval=abs(sin(theta_interval));
sup_sn_theta_interval=sn_theta_interval.sup;


H1 =[ 0, 0,           0,        0, 0;
      0, 0,           0,        0, 0;
      0, 0, sup_u1_interval*sup_cs_theta_interval, sup_sn_theta_interval, 0;
      0, 0,    sup_sn_theta_interval,   0, 0;
      0, 0,           0,        0, 0];
  
  
%   H1 =
%  
% [ 0, 0,           0,        0, 0]
% [ 0, 0,           0,        0, 0]
% [ 0, 0, -u1*cos(x3), -sin(x3), 0]
% [ 0, 0,    -sin(x3),        0, 0]
% [ 0, 0,           0,        0, 0]
  
  
 
 
H2 =[ 0, 0,           0,       0, 0;
      0, 0,           0,       0, 0;
      0, 0, sup_u1_interval*sup_sn_theta_interval, sup_cs_theta_interval, 0;
      0, 0,     sup_cs_theta_interval,       0, 0;
      0, 0,           0,       0, 0];
 
% H2= 
% [ 0, 0,           0,       0, 0]
% [ 0, 0,           0,       0, 0]
% [ 0, 0, -u1*sin(x3), cos(x3), 0]
% [ 0, 0,     cos(x3),       0, 0]
% [ 0, 0,           0,       0, 0]  
  
H3 =zeros(5,5);

  
  
end

