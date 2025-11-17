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

clearvars -except num_sim num_good_sim;
clear global;
close all;
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
global vehstate;        % the array containing both vehicles states: [dom_x, dom_y, dom_psi, sub_x, sub_y, sub_psi]
global dom_kormanyszog; % delta of dom. av.
global sub_kormanyszog; % delta of sub. av.
global dom_sebesseg;    % vx of dom. av.
global sub_sebesseg;    % vx of sub. av.
global pedestrian;      % [x,y] of the pedestrian
global dom_OK;
global sub_OK;
global dom_emergency;
global sub_emergency;
global dom_all_palya;
global sub_all_palya;
global t;
global dom_warnings;
global sub_warnings;
global dom_palya;
global sub_palya;
global dom_costmap;
global sub_costmap;
global danger;

% Initialize global variables
dom_init_pos = [1, 2.5, 0];
dom_goal_pos = [25, 2.5, 0];
sub_init_pos = [20, 7.5, 0];
sub_goal_pos = [4, 7.5, 0];
dom_vmax = 60;
sub_vmax = 40;
pedestrian = [10,2.5];


% Simulation parameters
Ts = 0.1;           % Sample time
steps = ceil(((dom_goal_pos(1)-dom_init_pos(1))/(dom_vmax/3.6))/Ts);     % Calculate this value: ceil(((goalX-startX)/v_x)/Ts)
T = 0:Ts:steps*Ts;  % The times belonging to the time indices
current_run = 0;    % number of simulation steps in the for loop

% Initialize some global variables
dom_all_palya = cell(steps,1);
sub_all_palya = cell(steps,1);
dom_warnings = {};
sub_warnings = {};
dom_emergency = false;
sub_emergency = false;
danger = false;

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
    current_run = current_run + 1



    % PATH PLANNER LAYER
    % First generate path for the dominant vehicle
    dom_palya = dom_palyagen(vehstate(t-1,:));

    % Then generate path for the subordinate vehicle
    sub_palya = sub_palyagen(vehstate(t-1,:),dom_palya); % THE CREATION OF THE FUNCTIONS ARE NEEDED

    

    % ACTIVATE EMERGENCY
    % If there's an error during the generation of the dominant palya, emergency scenario is true
    if ~dom_OK
        dom_emergency = true;
        % dom_warnings(end+1,:) = {1, 'Error', t-1, 'Vészhelyzet! A jármű egyenesen halad maximális fékezéssel!'};
    end

    % If there's an error during the generation of the subordinate palya, emergency scenario is true
    if ~sub_OK
        sub_emergency = true;
        % sub_warnings(end+1,:) = {1, 'Error', t-1, 'Vészhelyzet! A jármű egyenesen halad maximális fékezéssel!'};
    end



    % PATH TRACING LAYER
    % Lateral control of the dominant vehicle
    if ~dom_emergency
        % Optimize the steering angle of the vehicle
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb = -15*pi/180;
        ub = 15*pi/180;
        options = optimoptions('fmincon','Display','off');
        % based on the function created in dom_fun.m, the fmincon minimalizes the cost of that function, and returns the optimized delta
        [dom_delta,FVAL,EXITFLAG] = fmincon(@dom_fun,0,A,b,Aeq,beq,lb,ub,[],options);
        % the final steering angle (that goes to the actuator) is calculated (to smooth the steering angle)
        dom_korm = 0.8*dom_delta + 0.2*dom_kormanyszog(t-1);
    else
        dom_korm = 0;
    end

    % Longitudnal control of the dominant vehicle
    if dom_emergency
        disp('Vészhelyzet van!');
        % maximum deceleration is applied
        dom_seb = max(round(dom_sebesseg(t-1)*3.6)-3, 0.01);
    else
        % calculate the optimal velocity of the vehicle
        dom_seb = dom_sebopt(dom_korm);
    end

    
    % Lateral control of the subordinate vehicle
    if ~sub_emergency
        % Optimize the steering angle of the vehicle
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb = -15*pi/180;
        ub = 15*pi/180;
        options = optimoptions('fmincon','Display','off');
        % based on the function created in sub_fun.m, the fmincon minimalizes the cost of that funciton, and returns the optimized delta
        [sub_delta,FVAL,EXITFLAG] = fmincon(@sub_fun,0,A,b,Aeq,beq,lb,ub,[],options);
        % The final steering angle (that goes to the actuator) is calculated (to smooth the steering angle)
        sub_korm = 0.8*sub_delta + 0.2*sub_kormanyszog(t-1);
    else
        sub_korm = 0;
    end

    % Longitudinal control of the subordinate vehicle 
    if sub_emergency
        disp('Vészhelyzet van!');
        % maximum deceleration is applied
        sub_seb = max(round(sub_sebesseg(t-1)*3.6)-3,0.01);
    else
        % Calculate the optimal velocity of the vehicle
        sub_seb = sub_sebopt(sub_korm);
    end



    % REFRESH SS SYSTEM
    % DOMINANT VEHICLE
    % Refresh the states based on vehicle dynamics
    v_x = dom_seb/3.6;
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
    dom_vehsD = sysD.A*dom_vehsD + sysD.B*dom_korm;

    % calculate the elements of vehstate
    vehstate(t,3) = dom_vehsD(1);
    vehstate(t,1) = vehstate(t-1,1) + v_x*Ts*cos(vehstate(t,3));
    vehstate(t,2) = vehstate(t-1,2) + v_x*Ts*sin(vehstate(t,3));

    % Store the velocity and steering angle
    dom_sebesseg(t) = v_x;
    dom_kormanyszog(t) = dom_korm;


    % SUBORDINATE VEHICLE
    % Refresh the states based on vehicle dynamics
    v_x = sub_seb/3.6;
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
    sub_vehsD = sysD.A*sub_vehsD + sysD.B*sub_korm;

    % calculate the elements of vehstate
    vehstate(t,6) = sub_vehsD(1);
    vehstate(t,4) = vehstate(t-1,4) - v_x*Ts*cos(vehstate(t,6));
    vehstate(t,5) = vehstate(t-1,5) - v_x*Ts*sin(vehstate(t,6));

    % Store the velocity and steering angle
    sub_sebesseg(t) = v_x;
    sub_kormanyszog(t) = sub_korm;
end

figure;
plot(vehstate(:,1),vehstate(:,2), 'o-b');
xlim([0, 30]);
ylim([0,10]);
hold on;
plot(vehstate(:,4),vehstate(:,5), 'o-r');
hold off;
