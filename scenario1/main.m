%{
This is the main script to control the different simulations and evaluate them.
%}

clc;
clear;
clear global;

global OK;
global warnings sebesseg kormanyszog vehstate pedestrian;
global va_max vh_max;

OK = false;
num_sim = 0;
num_good_sim = 0;
simulations = 20;
all_warnings = {};
all_sebesseg = {};
all_kormanyszog = {};
all_vehstate = {};
all_vhmax = {};
all_vamax = {};
all_pedestrian = {};

% FOR RANDOMIZATION - the first elements are the default values
va_max_range = [60, 50, 40, 70];
vh_max_range = [40, 50, 60, 70];
pedestrian_x_range = [10, 11, 12, 13, 14, 15];
pedestrian_y_range = [2.5, 3, 3.5, 4];

rng("shuffle");

% for i=1:simulations
while ~OK
    num_sim = num_sim + 1;

    % Randomize initial pozitions
    va_max = va_max_range(randi(length(va_max_range)));
    vh_max = vh_max_range(randi(length(vh_max_range)));
    pedestrian = [pedestrian_x_range(randi(length(pedestrian_x_range))), pedestrian_y_range(randi(length(pedestrian_y_range)))];

    disp(['Simulation ', num2str(num_sim), ' initial values:']);
    disp(['va_max = ' num2str(va_max)]);
    disp(['vh_max = ' num2str(vh_max)]);
    disp(['pedestrian = [' num2str(pedestrian(1)), ', ' num2str(pedestrian(2)), ']']);
    disp(['Simulation ', num2str(num_sim), ' is running...']);
    run Simulation.m
    
    if ~isempty(warnings)
        OK = ~any(contains(warnings(:,2), 'Error'));
    else
        warnings(1,:) = {0, 'Info', 0, 'A szimuláció ideálisan lefutott.'};
    end

    % Store all the simulation data
    all_warnings{end+1,1} = warnings;
    all_sebesseg{end+1,1} = sebesseg;
    all_kormanyszog{end+1,1} = kormanyszog;
    all_vehstate{end+1,1} = vehstate;
    all_vhmax{end+1,1} = vh_max;
    all_vamax{end+1,1} = va_max;
    all_pedestrian{end+1,1} = pedestrian;

    if ~OK
        warning(['Hiba a ', num2str(num_sim), '. szimulációban.']);
    else
        num_good_sim = num_good_sim + 1;
        disp('A szimuláció sikeresen lefutott.')
    end
    disp(' ');
end

% disp(['Sikeres szimuláció. Szimulációk száma: ', num2str(num_sim)]);
disp(['Összes szimuláció: ', num2str(num_sim)]);
disp(['Jó szimulációk: ', num2str(num_good_sim)]);
disp(['A sikeres szimulációk aránya: ', num2str(num_good_sim/num_sim)]);
disp('Az ideálistól eltérő események:');
disp(warnings)

% Save all the simulation results
% save random_init_sim_results10.mat all_warnings all_sebesseg all_kormanyszog all_vehstate all_vhmax all_vamax all_pedestrian num_sim num_good_sim;