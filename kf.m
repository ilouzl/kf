function [x,P]=kf(F,x,B,u,P,H,z,Q,R)
% EKF   Kalman Filter for linear dynamic systems
% [x, P] = kf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for linear dynamic system:
% Model - x[k] = F*x[k-1]+B*u[k]+W*w[n]
%         w[n] ~ N(0,Q = W*cov(w[n])*W')
% Observation - z[k] = H*x[k]+V*v[n]
%               v[n] ~ N(0,R = V*cov(v[n])*V')
% Inputs:   F: dynamics matrix
%           x: "a priori" state estimate
%           B: control input matrix
%           u: control vector
%           P: "a priori" estimated state covariance
%           H: observation matrix
%           z: current observation
%           Q: process noise covariance 
%           R: observation noise covariance
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance
%
% note: X_1_1 <--> X_k-1|k-1, X1_1 <--> X_k|k-1, X11 <--> X_k|k
x_1_1 = x; P_1_1 = P;
x1_1 = F*x_1_1+B*u;
P1_1 = F*P_1_1*F'+Q;
y = z - H*x1_1;
S = H*P1_1*H'+R;
K = P1_1*H'*inv(S);
x11 = x1_1+K*y;
P11 = (eye(size(P1_1,1))-K*H)*P1_1;
P = P11;
x = x11;