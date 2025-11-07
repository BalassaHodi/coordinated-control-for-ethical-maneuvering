
%{
This function implements the longitudinal control design of the av. during
the simulation.
The constaints of the velocity is that in every time step it can only
change by 3 km/h.
The approach here is that the veolcity shall remain the same (so no
braking) unless with the higher speed the car goes into an inflated area
(so the higher speed is not safe).
%}


function seb = sebopt(delta)

% Global variables
global OK;
global vehsD vehstate t;
global palya;
global costmap sebesseg;
global va_max;


% Vehicles constant parameters
C1 = 80000;     % cornering stiffness of front tires
C2 = 120000;    % cornering stiffness of back tires
l1 = 2.2;       % distance between COG and front axle
l2 = 2.3;       % distance between COG and back axle
J = 2500;       % inertia around axis z of the vehicle
m = 1500;       % mass of the vehicle

% Sample time
Ts = 0.1;


% The for loop goes through the velocties from act_vel+3 to act_vel-3 (in km/h)
for seb = min(va_max, round(sebesseg(t-1)*3.6) + 3):-1:max(0, round(sebesseg(t-1)*3.6) - 3)
    % If the velocity is 0, than it shall be 0.01
    if seb < 0.1
        seb = 0.01;
        break
    end

    % Update the state space with the velocity
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

    % Update the local vector of states
    vehsD_ = sysD.A*vehsD + sysD.B*delta;

    % Update the states of the av.
    vehstate_(t,3) = vehsD_(1);
    vehstate_(t,1) = vehstate(t-1,1) + v_x*Ts*cos(vehstate_(t,3));
    vehstate_(t,2) = vehstate(t-1,2) + v_x*Ts*sin(vehstate_(t,3));

    % Check wether the velocity is good or not
    % The angle shall be converted into degrees and according to the documentation it is positive in the clockwise direction
    if checkFree(costmap,[vehstate_(t,1), vehstate_(t,2), -vehstate_(t,3)*180/pi]) == 1
        % If good, then break the loop and the output will be the velocity
        break
    end
end