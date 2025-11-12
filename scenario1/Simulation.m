%{
This script runs the simulation for every timestep.
%}

clearvars -except num_sim num_good_sim all_warnings all_sebesseg all_kormanyszog all_vehstate all_vh all_pedestrian;
close all;
% clc;

% Global variables
global OK;          % [boolean] variable for debugging. True if path was created well, false if not.
global vehsD;       % [n-by-1 double] (n: states of the discrete state space model: [psi, v_y, y])
global vehstate;    % [n-by-m double] (n: time steps) (m: states of av. and hv.: [x_av, y_av, psi_av, x_hv])
global t;           % [double] (time index (the index of a given time step))
global palya;       % [n-by-m array] (n: time steps) (m: the poses of the reference route from the RRT: [x, y, psi, length])
global costmap;     % [vehicleCostmap] (the vehicle costmap representing the planning search space around the av.)
global kormanyszog; % [1-by-m double] (m: the steering angles actually given to the actuator in radians in each time step)
global sebesseg;    % [1-by-m double] (m: the actual velocity of the av. in m/s in each time step)
global va_max;      % [double] the maximum velocity of the av. in m/s
global all_palya;   % [n-by-1 cell] (n: the timestep that contains the palya array)
global warnings;    % [n-by-4 cell] (n: the number of warnings in a single simulation) (the columns: Code, Type, TimeStep, Description)
global emergency;   % [boolean] (true if the emergency situation happens (go straight with max deceleration)
global vh;
global pedestrian;

% The steps
steps = 15;

% Initialize global variables
all_palya = cell(steps,1);
warnings = {};
emergency = false;
va_max = 60;

% Vehicles constant parameters
C1 = 80000;     % cornering stiffness of front tires
C2 = 120000;    % cornering stiffness of back tires
l1 = 2.2;       % distance between COG and front axle
l2 = 2.3;       % distance between COG and back axle
J = 2500;       % inertia around axis z of the vehicle
m = 1500;       % mass of the vehicle

% Simulation constant parameters
Ts = 0.1;           % Sample time
T = 0:Ts:steps*Ts;  % The times belonging to the time indexes

% State space equation
% x_dot = A * [ddot_psi; dot_vy; vy] + B * delta;


% Number of simulation step in the foor loop (displayed on command window)
current_run = 0;

% Initial values
va = va_max/3.6;                 % the maximum velocity of the av. in m/s
vh = 40/3.6;                     % human-driven vehicle velocity in m/s (for the whole simulation remains the same)
vehsD = [0; 0; 0];               % initial condition
vehstate(1,:) = [1, 2.5, 0, 15]; % the av. is in the begining of the route and in the middle of its lane, ...
                                 % the hv. is at the opposite end of the opposite lane
kormanyszog(1) = 0;              % the initial steering angle in radians
sebesseg(1) = va;                % av. initial velocity in m/s



% The simulation (loop that runs for every timestep)
for t = 2:length(T)
    % The number of the current simulation step (display it to cmd wdw if needed)
    current_run = current_run + 1;

    % Generate the path that the av. should follow
    palya = palyagen(vehstate(t-1,:));

    % If there's an error during the generation of the "palya", emergency scenario is true
    if ~OK
        emergency = true;
        warnings(end+1,:) = {1, 'Error', t-1, 'Vészhelyzet! A jármű egyenesen halad maximális fékezéssel!'};
    end
    

    % Refresh the position of the hv.
    vehstate(t,4) = vehstate(t-1,4) - Ts*vh;
    
    if ~emergency
        % Optimize the steering angle of the av.
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb = -15*pi/180;    % the lower bound of the steering angle
        ub=15*pi/180;       % the upper bound of the steering angle
        options = optimoptions('fmincon','Display','off');
        % based on the function created in 'fun.m', the fmincon minimalizes the cost of that function, and returns the optimized delta
        [delta,FVAL,EXITFLAG] = fmincon(@fun,0,A,b,Aeq,beq,lb,ub,[],options);
        % the final steering angle (that goes to the actuator) is calculated (to smooth the steering angle)
        korm = 0.8*delta + 0.2*kormanyszog(t-1);
    else
        korm = 0;
    end

    % Optimize the velocity of the vehicle
    if emergency
        % disp('Vészhelyzet van!');
        % maximum deceleration is applied
        seb = max(round(sebesseg(t-1)*3.6)-3,0.01);
    else
        % calculate the optimal velocity of the vehicle
        seb = sebopt(korm);
    end


    % refresh the states based on the vehicle dynamics
    v_x = seb/3.6;
    A = [(-C1*l1^2-C2*l2^2)/(J*v_x), (-C1*l1+C2*l2)/(J*v_x), 0;
         (-C1*l1+C2*l2)/(m*v_x)-v_x, (-C1-C2)/(m*v_x), 0;
         0, 1, 0];
    B = [C1*l1/J; C1/m; 0];
    C = [0 0 1;
        1 0 0];
    D = [0;0];
    sysC = ss(A,B,C,D);
    sysD = c2d(sysC,Ts);
    
    % refresh the global vector of states of the discrete ss system
    vehsD = sysD.A*vehsD + sysD.B*korm;
    
    % calculate the elements of the state of the av. based on vehsD
    vehstate(t,3) = vehsD(1);   % psi [rad]
    vehstate(t,1) = vehstate(t-1,1) + v_x*Ts*cos(vehstate(t,3));    % x_av [m/s]
    vehstate(t,2) = vehstate(t-1,2) + v_x*Ts*sin(vehstate(t,3));    % y_av [m/s]

    
    % Store the velocity and steering angle
    sebesseg(t) = v_x;
    kormanyszog(t) = korm;
end



% Create the figure of the velocity profile, and the steering angle over time
% And also create the actual path of the car
% if OK
%     run plot_simit.m
%     figure;
%     plot(vehstate(:,1),vehstate(:,2), 'o-b');
%     xlim([0, 30]);
%     ylim([0,10]);
% end
