%{
This is the main script to control the different simulations and evaluate them.
%}

clc;
clear;

global OK;
global warnings sebesseg kormanyszog vehstate vh pedestrian;
% global all_warnings all_sebesseg all_kormanyszog all_vehstate all_vh all_pedestrian;

OK = false;
num_sim = 0;
num_good_sim = 0;
simulations = 10;
all_warnings = {};
all_sebesseg = {};
all_kormanyszog = {};
all_vehstate = {};
all_vh = {};
all_pedestrian = {};

for i=1:simulations
%while ~OK
    num_sim = num_sim + 1;
    disp(['Simulation ', num2str(num_sim), ' is running...'])
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
    all_vh{end+1,1} = vh;
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
% disp('Az ideálistól eltérő események:');
% disp(warnings)

% Save all the simulation results
% save all_simulation_results.mat all_warnings all_sebesseg all_kormanyszog all_vehstate all_vh all_pedestrian num_sim num_good_sim;