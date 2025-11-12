% SCENARIO 2
%{
All the variables containing 'dom' are for the dominant/superior vehicle.
All the variables containing 'sub' are for the subordinate vehicle.

The superior vehicle is the one who gets into danger first, the subordinate
is the one which helps him avoid the collusion.

For easier calculations the superior vehicle for us is always in the right lane,
and the subordinate is always at the left lane.
%}

%{
This script runs the simulation for every timestep.
%}

clear;
clear global;
clc;

% Global variables
global steps;           % the steps of one simulation
global dom_init_pos;    % The initial position of the dominant vehicle
global dom_goal_pos;    % The goal position of the dominant vehicle
global sub_init_pos;    % The initial position of the subordinate vehicle
global sub_goal_pos;    % The goal position of the subordinate vehicle
global dom_vmax;        % The maximum velocity of the dominant vehicle
global sub_vmax;        % the maximum velcity of the subordinate vehicle
global dom_vehsD;       % the states of the discrete ss model for the dominant vehicle
global sub_vehsD;       % the states of the discrete ss model for the subordinate vehicle
global vehstate;       % the array containing both vehicles states: [dom_x, dom_y, dom_psi, sub_x, sub_y, sub_psi]
global dom_kormanyszog; % delta of dom. av.
global sub_kormanyszog; % delta of sub. av.
global dom_sebesseg;    % vx of dom. av.
global sub_sebesseg;    % vx of sub. av.


% Initialize global variables
dom_init_pos = [1, 2.5, 0];
dom_goal_pos = [25, 2.5, 0];
sub_init_pos = [20, 7.5, 0];
sub_goal_pos = [5, 7.5, 0];
dom_vmax = 60;
sub_vmax = 40;


% Simulation parameters
Ts = 0.1;           % Sample time
steps = ceil(((dom_goal_pos(1)-dom_init_pos(1))/(dom_vmax/3.6))/Ts);     % Calculate this value: ceil(((goalX-startX)/v_x)/Ts)
T = 0:Ts:steps*Ts;  % The times belonging to the time indices
current_run = 0;    % number of simulation steps in the for loop


% Vehicles constant parameters - for both av.
C1 = 80000;     % cornering stiffness of front tires
C2 = 120000;    % cornering stiffness of back tires
l1 = 2.2;       % distance between COG and front axle
l2 = 2.3;       % distance between COG and back axle
J = 2500;       % inertia around axis z of the vehicle
m = 1500;       % mass of the vehicle


% Initial values
dom_v = dom_vmax/3.6;       % dom. init vel. in m/s
sub_v = sub_vmax/3.6;      % sub. init vel. in m/s
dom_vehsD = [0;0;0];        % [psi_dot, vy, y]
sub_vehsD = [0;0;0];        % [psi_dot, vy, y]
vehstate(1,:) = [dom_init_pos, sub_init_pos]; % [dom_x, dom_y, dom_psi, sub_x, sub_y, sub_psi]
dom_kormanyszog(1) = 0;     % in rad
sub_kormanyszog(1) = 0;     % in rad
dom_sebesseg(1) = dom_v;    % in m/s
sub_sebesseg(1) = sub_v;    % in m/s


% The simulation (loop that runs for every timestep)
for t = 2:length(T)
    % The number of the current simulation step (display it to cmd wdw if needed)
    current_run = current_run + 1;

    % JUST FOR DEBUGGING NOW
    % refresh the states based on the vehicle dynamics
    % FIRST for the dominant vehicle
    seb = dom_v;
    korm = 0;
    v_x = seb;
    A = [(-C1*l1^2-C2*l2^2)/(J*v_x), (-C1*l1+C2*l2)/(J*v_x), 0;
         (-C1*l1+C2*l2)/(m*v_x)-v_x, (-C1-C2)/(m*v_x), 0;
         0, 1, 0];
    B = [C1*l1/J; C1/m; 0];
    C = [0 0 1;
        1 0 0];
    D = [0;0];
    sysC = ss(A,B,C,D);
    sysD = c2d(sysC,Ts);
    % refresh the vector of states for the dominant discrete ss system
    dom_vehsD = sysD.A*dom_vehsD + sysD.B*korm;

    % calculate the elements of vehstate
    vehstate(t,3) = dom_vehsD(3);
    vehstate(t,1) = vehstate(t-1,1) + v_x*Ts*cos(vehstate(t,3));
    vehstate(t,2) = vehstate(t-1,2) + v_x*Ts*sin(vehstate(t,3));

    % SECOND for the subordinate vehicle
    seb = sub_v;
    korm = 0;
    v_x = seb;
    A = [(-C1*l1^2-C2*l2^2)/(J*v_x), (-C1*l1+C2*l2)/(J*v_x), 0;
         (-C1*l1+C2*l2)/(m*v_x)-v_x, (-C1-C2)/(m*v_x), 0;
         0, 1, 0];
    B = [C1*l1/J; C1/m; 0];
    C = [0 0 1;
        1 0 0];
    D = [0;0];
    sysC = ss(A,B,C,D);
    sysD = c2d(sysC,Ts);
    % refresh the vector of states for the sub. discrete ss system
    sub_vehsD = sysD.A*sub_vehsD + sysD.B*korm;

    % calculate the elements of vehstate
    vehstate(t,6) = sub_vehsD(3);
    vehstate(t,4) = vehstate(t-1,4) - v_x*Ts*cos(vehstate(t,6));
    vehstate(t,5) = vehstate(t-1,5) - v_x*Ts*sin(vehstate(t,6));
end

figure;
plot(vehstate(:,1),vehstate(:,2), 'o-b');
xlim([0, 30]);
ylim([0,10]);
hold on;
plot(vehstate(:,4),vehstate(:,5), 'o-r');
hold off;