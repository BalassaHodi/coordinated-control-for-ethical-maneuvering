% SCENARIO 2

%{
This function implements the longitudinal control design of the dominant vehicle
during the simulation.
The constaints of the velocity is that in every time step it can only
change by 3 km/h.
The approach here is that the veolcity shall remain the same (so no
braking) unless with the higher speed the car goes into an inflated area
(so the higher speed is not safe).
---
The input:
- a delta value: [double] steering angle
- a seb value: [double] a longitudinal velocity value
%}


function sub_seb = sub_sebopt(delta)

% Global variables
global sub_vmax;
global sub_sebesseg;
global t;
global sub_vehsD;
global vehstate;
global sub_costmap;


% Vehicle constant parameters
C1 = 80000;     % cornering stiffness of front tires
C2 = 120000;    % cornering stiffness of back tires
l1 = 2.2;       % distance between COG and front axle
l2 = 2.3;       % distance between COG and back axle
J = 2500;       % inertia around axis z of the vehicle
m = 1500;       % mass of the vehicle

% Sample time
Ts = 0.1;

% The for loop goes through the velocities from act_vel+3 to act_vel-3 (in km/h)
for sub_seb = min(sub_vmax, round(sub_sebesseg(t-1)*3.6)+3):-1:max(0, round(sub_sebesseg(t-1)*3.6)-3)
    % If the velocity is 0, then it shall be 0.01
    if sub_seb < 0.1
        sub_seb = 0.01;
        break
    end

    % Update the state space with the velocity
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

    % Update the local vector of states
    vehsD_ = sysD.A*sub_vehsD + sysD.B*delta;

    % Update the local array of states of the subordinate vehicle
    vehstate_(t,3) = vehsD_(1);
    vehstate_(t,1) = vehstate(t-1,4) - v_x*Ts*cos(vehstate_(t,3));
    vehstate_(t,2) = vehstate(t-1,5) - v_x*Ts*sin(vehstate_(t,3));

    % Check wether the velocity is good or not
    if checkFree(sub_costmap,[vehstate_(t,1), vehstate_(t,2), -(vehstate_(t,3)*180/pi+180)]) == 1
        % If good, then break the loop and the output will be the velocity
        break
    end
end