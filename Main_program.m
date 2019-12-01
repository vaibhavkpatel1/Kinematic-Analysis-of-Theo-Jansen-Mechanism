%% -------------------------- INTRODUCTION ------------------------------
%
% Main_program.m
% Version 1.0
% Created by Vaibhav Patel
%
% This codes is written for the Kinematics and Dynamics of Machinery

% This Code is animation of  Jansen Mechanism, a mechanism widely 
% used in Robotics to mimic behaviour of legged Robots. The problem 
% statement it satisfies is : 1 Animate the motion of the mechanism 
% when the input link moves within its range of motion. 
% The initial configuration maybe assumed.

% The variables used to represent link number represented in the code 
% corresponds to the diagram which is present in the file along with 
% this code. A Report explaining Code is also accompnied.

%--------------------------------------------------------------------

%% --------------------POSITION ANALYSIS-------------------------------

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

tht_i = 0;
tht_1 = 2.4609;
tht_2 = 1.2217;
tht_3 = 3.5779;
tht_4 = 2.7401;
tht_5 = 5.1138;
tht_6 = 3.9618;
tht_7 = 4.9916;
tht_8 = 2.6529;
tht_9 = 4.2807;
tht_10 = 1.8151;

%% Number of Iterations and Time Step

t = 100;
dt = 2 * pi / t;

%% Loop Iterations
% Note - Newton-Raphson Method is not applied because it leads to
%        significant Errors and therefore wrong Results.

for i = 1:t
    
    %% Jacobian Matrix and RHS matrix for Angular Velocity
    J1 = [ -l1*cos(tht_1), l2*cos(tht_2);
           -l1*sin(tht_1), l2*sin(tht_2);];

    RHS_J1_v = [li*cos(tht_i); 
                li*sin(tht_i);];

    J2 = [ -l6*cos(tht_6),  l7*cos(tht_7);
            l6*sin(tht_6), -l7*sin(tht_7);];
    
    RHS_J2_v = [ li*cos(tht_i);
                -li*sin(tht_i);];

    J3 = [  l8*cos(tht_8), -l5*cos(tht_5);
            l8*sin(tht_8), -l5*sin(tht_5);];

    RHS_J3_v = [  l4*cos(tht_4), -l7*cos(tht_7);
                  l4*sin(tht_4), -l7*sin(tht_7);]; 
    
    %% ------------------- Angular Velocity Analysis --------------------
   
    % Asuming Angular Velocity of Crank to be 1 and fixed link to be 0
    omega_i = 1;
    
    % Angular Velocity for Link 1 and 2
    temp = (J1 \ RHS_J1_v) * omega_i; 
    omega_1 = temp(1);
    omega_2 = temp(2);
    omega_4 = omega_2;          % Common Centre and fixed angle
    
    % Angular Velocity for Link 6 and 7
    temp = (J2 \ RHS_J2_v) * omega_i; 
    omega_6 = temp(1);
    omega_7 = temp(2);
    
    % Angular Velocity for Link 8 and 5
    temp = (J3 \ RHS_J3_v) * [omega_4; omega_7]; 
    omega_8 = temp(1);
    omega_5 = temp(2);
    
    tht_i = tht_i + omega_i * dt;                   % Input Link (cranck)
    tht_1 = tht_1 + omega_1 * dt;   
    tht_2 = tht_2 + omega_2 * dt; 
    tht_4 = tht_4 + omega_4 * dt;
    tht_5 = tht_5 + omega_5 * dt;
    tht_6 = tht_6 + omega_6 * dt;
    tht_7 = tht_7 + omega_7 * dt;
    tht_8 = tht_8 + omega_8 * dt;
    tht_9 = tht_9 + omega_8 * dt;
    
    %% Plotting on the Matlab Graph 
    
    clf; hold on;
    xlim([-150 150])
    ylim([-150 150])
    O1 = [0,0];
    Ri = li * [cos(tht_i),sin(tht_i)] + [b,a];
    R1 = Ri + l1 * [cos(tht_1),sin(tht_1)];
    R2 = l2 * [cos(tht_2),sin(tht_2)];
    R3 = R2 + l3 * [cos(tht_3),sin(tht_3)];
    R4 = l4 * [cos(tht_4),sin(tht_4)];
    R5 = R4 + l5 * [cos(tht_5),sin(tht_5)];
    R6 = Ri + l6 * [cos(tht_6),sin(tht_6)];
    R7 = l7 * [cos(tht_7),sin(tht_7)];
    R8 = R7 + l8 * [cos(tht_8),sin(tht_8)];
    R9 = R7 + l9 * [cos(tht_9),sin(tht_9)];
    
    A_x(i) = Ri(1); A_y(i) = Ri(2);
    B_x(i) = R1(1); B_y(i) = R1(2);
    C_x(i) = R4(1); C_y(i) = R4(2);
    D_x(i) = R7(1); D_y(i) = R7(2);
    E_x(i) = R8(1); E_y(i) = R8(2);
    P_x(i) = R9(1); P_y(i) = R9(2);
    
    plot([b,Ri(1)],[a,Ri(2)],'-b','Linewidth',4);
    plot([O1(1),R2(1)],[O1(2),R2(2)],'-r','Linewidth',4);
    plot([Ri(1),R1(1)],[Ri(2),R1(2)],'-g','Linewidth',4);
    plot([O1(1),R4(1)],[O1(2),R4(2)],'-r','Linewidth',4);
    plot([O1(1),R7(1)],[O1(2),R7(2)],'-r','Linewidth',4);
    plot([Ri(1),R7(1)],[Ri(2),R7(2)],'-b','Linewidth',4);
    plot([R4(1),R5(1)],[R4(2),R5(2)],'-g','Linewidth',4);
    plot([R7(1),R8(1)],[R7(2),R8(2)],'-b','Linewidth',4);
    plot([R2(1),R4(1)],[R2(2),R4(2)],'-b','Linewidth',4);
    plot([R7(1),R9(1)],[R7(2),R9(2)],'-b','Linewidth',4);
    plot([R8(1),R9(1)],[R8(2),R9(2)],'-b','Linewidth',4);
    
    plot(O1(1),O1(2),'o','Markersize',12);
    plot(Ri(1),Ri(2),'o','Markersize',12);
    plot(R1(1),R1(2),'o','Markersize',12);
    plot(R2(1),R2(2),'o','Markersize',12);
    plot(R4(1),R4(2),'o','Markersize',12);
    plot(R5(1),R5(2),'o','Markersize',12);
    plot(R6(1),R6(2),'o','Markersize',12);
    plot(R7(1),R7(2),'o','Markersize',12);
    plot(R8(1),R8(2),'o','Markersize',12);
    plot(R9(1),R9(2),'o','Markersize',12);
    
    pause(0.05);
    
end

figure; hold on; grid on;
plot(P_x,P_y);
title('Locus of point P');
xlabel('X Coordinate of P');
ylabel('Y Coordinate of P');

figure; hold on; grid on;
plot(A_x,A_y);
title('Locus of point 1');
xlabel('X Coordinate of 1');
ylabel('Y Coordinate of 1');

figure; hold on; grid on;
plot(B_x,B_y);
title('Locus of point 2');
xlabel('X Coordinate of 2');
ylabel('Y Coordinate of 2');

figure; hold on; grid on;
plot(C_x,C_y);
title('Locus of point 4');
xlabel('X Coordinate of 4');
ylabel('Y Coordinate of 4');

figure; hold on; grid on;
plot(D_x,D_y);
title('Locus of point 5');
xlabel('X Coordinate of 5');
ylabel('Y Coordinate of 5');

figure; hold on; grid on;
plot(E_x,E_y);
title('Locus of point 6');
xlabel('X Coordinate of 6');
ylabel('Y Coordinate of 6');