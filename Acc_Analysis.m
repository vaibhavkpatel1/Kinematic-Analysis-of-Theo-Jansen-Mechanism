%% ------------------------ Acceleration Analysis -----------------------
%
% Acc_Analysis.m
% Version 1.0
% Created by Vaibhav Patel
%
% This code presents Acceleration Analysis for Jansen Mechanism.

%% Length of Links (in mm) are defined in term of l1. 

li = 20;                % Input Link (cranck)
l1 = 3.33 * li;
l2 = 2.77 * li;
l3 = 3.72 * li;
l4 = 2.67 * li;
l5 = 2.63 * li;
l6 = 4.13 * li;
l7 = 2.62 * li;
l8 = 2.45 * li;
l9 = 3.27 * li;
l10 = 4.38 * li;
a = 0.52 * li;          % Ground Link
b = 2.53 * li;          % Ground Link and Origin is at this link's joint

%% Angles of Link with respect to Ground

tht_i(1) = 0;
tht_1(1) = 2.4609;
tht_2(1) = 1.2217;
tht_3(1) = 3.5779;
tht_4(1) = 2.7401;
tht_5(1) = 5.1138;
tht_6(1) = 3.9618;
tht_7(1) = 4.9916;
tht_8(1) = 2.6529;
tht_9(1) = 4.2807;
tht_10(1) = 1.8151;

%% Number of Iterations and Time Step

t = 50;
dt = 2 * pi / t;

%% Loop for Calculating all Accelerations

for i = 1:t
    
            %% Jacobian Matrix and RHS matrix for Angular Velocity 
    
    J1 = [ -l1*cos(tht_1(i)), l2*cos(tht_2(i));       % Jacobian Loop 1
           -l1*sin(tht_1(i)), l2*sin(tht_2(i));];
    
    RHS_J1_v = [li*cos(tht_i(i)); 
                li*sin(tht_i(i));];
        
    J2 = [ -l6*cos(tht_6(i)),  l7*cos(tht_7(i));      % Jacobian Loop 2
            l6*sin(tht_6(i)), -l7*sin(tht_7(i));];
    
    RHS_J2_v = [ li*cos(tht_i(i));
                -li*sin(tht_i(i));];
      
    J3 = [  l8*cos(tht_8(i)), -l5*cos(tht_5(i));      % Jacobian Loop 3
            l8*sin(tht_8(i)), -l5*sin(tht_5(i));];
    
    RHS_J3_v = [  l4*cos(tht_4(i)), -l7*cos(tht_7(i));
                  l4*sin(tht_4(i)), -l7*sin(tht_7(i));]; 
   
    % Asuming Angular Velocity of Crank to be 1 and fixed link to be 0
    omega_i = 1;
    
    % Angular Velocity for Link 1 and 2
    temp = (J1 \ RHS_J1_v) * omega_i; 
    omega_1(i) = temp(1);
    omega_2(i) = temp(2);
    omega_4(i) = omega_2(i);          % Common Centre and fixed angle
    
    % Angular Velocity for Link 6 and 7
    temp = (J2 \ RHS_J2_v) * omega_i; 
    omega_6(i) = temp(1);
    omega_7(i) = temp(2);
    
    % Angular Velocity for Link 8 and 5
    temp = (J3 \ RHS_J3_v) * [omega_4(i); omega_7(i)]; 
    omega_8(i) = temp(1);
    omega_5(i) = temp(2);
     
    %% Jacobian Matrix and RHS matrix for Angular Acceleration
    
    RHS_J1_a = [ -li*omega_i*sin(tht_i(i)) - l1*omega_1(i)*sin(tht_1(i)) - ...
                  l2*omega_2(i)*sin(tht_2(i));
                 -li*omega_i*cos(tht_i(i)) + l1*omega_1(i)*cos(tht_1(i)) - ...
                  l2*omega_2(i)*cos(tht_2(i));];
              
    % Angular Acceleration for Link 1 and 2
    
    temp = (J1 \ RHS_J1_a); 
    alpha_1(i) = temp(1);
    alpha_2(i) = temp(2);
    alpha_4(i) = alpha_2(i);                   % Common Centre and fixed angle     

    RHS_J2_a = [ -li*omega_i*sin(tht_i(i)) - l6*omega_6*sin(tht_6(i)) + ...
                  l7*omega_7*sin(tht_7(i));
                  li*omega_i*cos(tht_i(i)) - l6*omega_6*cos(tht_6(i)) + ...
                  l7*omega_7*cos(tht_7(i));];
          
    % Angular Acceleration for Link 6 and 7
    temp = (J2 \ RHS_J2_a); 
    alpha_6(i) = temp(1);
    alpha_7(i) = temp(2);          
          
    RHS_J3_a = [ l4*alpha_4(i)*cos(tht_4(i)) - l4*omega_4(i)*sin(tht_4(i)) - ...
                 l7*alpha_7(i)*cos(tht_7(i)) + l7*omega_7(i)*sin(tht_7(i)) + ...
                 l8*omega_8(i)*sin(tht_8(i)) - l5*omega_5(i)*sin(tht_5(i));
                 l4*alpha_4(i)*sin(tht_4(i)) + l4*omega_4(i)*cos(tht_4(i)) - ...
                 l7*alpha_7(i)*sin(tht_7(i)) - l7*omega_7(i)*cos(tht_7(i)) + ...
                -l8*omega_8(i)*cos(tht_8(i)) + l5*omega_5(i)*cos(tht_5(i)); ];

    % Angular Acceleration for Link 8 and 5
    temp = (J3 \ RHS_J3_a); 
    alpha_8(i) = temp(1);
    alpha_5(i) = temp(2);
    
    tht_i(i+1) = tht_i(i) + omega_i * dt;                   % Input Link (cranck)
    tht_1(i+1) = tht_1(i) + omega_1(i) * dt;   
    tht_2(i+1) = tht_2(i) + omega_2(i) * dt; 
    tht_4(i+1) = tht_4(i) + omega_4(i) * dt;
    tht_5(i+1) = tht_5(i) + omega_5(i) * dt;
    tht_6(i+1) = tht_6(i) + omega_6(i) * dt;
    tht_7(i+1) = tht_7(i) + omega_7(i) * dt;
    tht_8(i+1) = tht_8(i) + omega_8(i) * dt;
    
    tht_input(i) = tht_i(i);
    
end

%% Plotting Various Graphs

figure; hold on; grid on;
plot(tht_input,alpha_1);
title('\alpha_j vs \theta_m');
xlabel('\theta_m \rightarrow');
ylabel('\alpha_j \rightarrow');

figure; hold on; grid on;
plot(tht_input,alpha_2);
title('\alpha_{\Deltabde} vs \theta_m');
xlabel('\theta_m \rightarrow');
ylabel('\alpha_{\Deltabde} \rightarrow');

% figure; hold on; grid on;
% plot(tht_input,alpha_4);
% title('\alpha_4 vs \theta_m');
% xlabel('\theta_m \rightarrow');
% ylabel('\alpha_4 \rightarrow');

figure; hold on; grid on;
plot(tht_input,alpha_5);
title('\alpha_f vs \theta_m');
xlabel('\theta_m \rightarrow');
ylabel('\alpha_f \rightarrow');

figure; hold on; grid on;
plot(tht_input,alpha_6);
title('\alpha_k vs \theta_m');
xlabel('\theta_m \rightarrow');
ylabel('\alpha_k \rightarrow');

figure; hold on; grid on;
plot(tht_input,alpha_7);
title('\alpha_c vs \theta_m');
xlabel('\theta_m \rightarrow');
ylabel('\alpha_c \rightarrow');

figure; hold on; grid on;
plot(tht_input,alpha_8);
title('\alpha_{\Deltaghi} vs \theta_m');
xlabel('\theta_m \rightarrow');
ylabel('\alpha_{\Deltaghi} \rightarrow');