% SCENARIO 2

%{
This function implements the lateral control design of the subordinate vehicle
during the simulation.
The desired steering angle (sub_delta) is calculated by minimalizing a 
pre-defined cost function.
The cost function that has to be minimized is the following:
J = (y_ref - y)^2 + 1.7 * (prev_delta - delta)^2
---
The input:
- a delta value: [double] that is the steering angle
The output:
- a cost function: That has to be minimized with the changing of 'delta'
%}

function kimenet = sub_fun(delta)

% Global variables
global sub_kormanyszog;
global sub_sebesseg;
global sub_vehsD;
global vehstate;
global sub_palya;
global t;
global sub_warnings;

% Vehicle constant parameters
C1 = 80000;     % cornering stiffness of front tires
C2 = 120000;    % cornering stiffness of back tires
l1 = 2.2;       % distance between COG and front axle
l2 = 2.3;       % distance between COG and back axle
J = 2500;       % inertia around axis z of the vehicle
m = 1500;       % mass of the vehicle

% Sample time
Ts = 0.1;

% The previous steering angle
korm = 0.8*delta + 0.2*sub_kormanyszog(t-1);

% Update the state space system
v_x = sub_sebesseg(t-1);
A = [(-C1*l1^2-C2*l2^2)/(J*v_x), (-C1*l1+C2*l2)/(J*v_x), 0;
     (-C1*l1+C2*l2)/(m*v_x)-v_x, (-C1-C2)/(m*v_x), 0;
     0, 1, 0];
B = [C1*l1/J; C1/m; 0];
C = [0 0 1;
    1 0 0];
D = [0;0];
sysC = ss(A,B,C,D);
sysD = c2d(sysC,Ts);

% A local variable for the vector of states
vehsD_ = sysD.A*sub_vehsD + sysD.B*korm;

% A local variable for the states of the subordinate vehicle
vehstate_(t,3) = vehsD_(1);
vehstate_(t,1) = vehstate(t-1,4) - v_x*Ts*cos(vehstate_(t,3));
vehstate_(t,2) = vehstate(t-1,5) - v_x*Ts*sin(vehstate_(t,3));

% Interpolate the y_ref value from the subordinate palya
y_ref = interp1(sub_palya(:,1), sub_palya(:,2), vehstate_(t,1));
if ~isfinite(y_ref)
    y_ref = sub_palya(2,2);
    sub_warnings(end+1,:) = {'SUB', 8, 'Info', t-1, 'Az interpoláció nem volt sikeres az y_ref kiszámításához.'};
end

% The cost function that has to be minimized
kimenet = (y_ref - vehstate_(t,2))^2 + 1.7*(korm-sub_kormanyszog(t-1))^2;