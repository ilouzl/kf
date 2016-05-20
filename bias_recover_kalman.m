close all; clearvars; clc;


% simulation length 
Fs = 5000; T = 0.1; t = 0:1/Fs:T;
s = [1 -1 0];

% Model - x[k] = F*x[k-1]+B*u[k]+W*w[n]
%         w[n] ~ N(0,Q = W*cov(w[n])*W')
F = [1 0 0 0; ...
     0 1 0 0; ...
     0 0 1 0; ...
     0 0 0 1]; 
B = [(1/Fs); 0; (1/Fs)/s(2); 0]; 
u = 90; 
W = [1;1;0;0]; 
sigma_w = 0;
Q = W*W'*sigma_w^2;

% Observation - z[k] = H*x[k]+V*v[n]
%               v[n] ~ N(0,R = V*cov(v[n])*V')
H = [0 0 1 1]; 
V = 1; 
sigma_v = 1;
R = V*V'*sigma_v^2;


% initialization
x = [0;1000;1000;0.3];         % initial state
x11 = [0;1000;1000;0];           % assume we know the initial position
P11 = diag([0,0,0,1]);  % assume initial estimation is correct
filt = []; true = []; meas = []; P = [];
for i=1:length(t)
    x_1_1 = x11; P_1_1 = P11;
    w = sigma_w*randn(1); v = sigma_v*randn(1); 
    x = F*x+B*u+W*w;
    z = H*x+V*v;
    [x11,P11]=kf(F,x_1_1,B,u,P_1_1,H,z,Q,R);
    P = [P P11([1 4])'];
    filt = [filt x11];
    true = [true x];
    meas = [meas z];
end
subplot(3,1,1);
plot(t,true(1,:),t,filt(1,:)); 
legend({'true','meas','est'});
subplot(3,1,2);
plot(t,true(4,:),t,filt(4,:)); 
subplot(3,1,3);
plot(t,meas(1,:)); 