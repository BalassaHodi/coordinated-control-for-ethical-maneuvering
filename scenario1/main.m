%{
This is the main script to control the different simulations and evaluate them.
%}

clc;
clear;

global OK;
global warnings;
global all_warnings;

OK = false;
num_sim = 0;
num_good_sim = 0;
simulations = 5;
all_warnings = {};

for i=1:simulations
%while ~OK
    num_sim = num_sim + 1;
    run Simulation.m
    
    if ~isempty(warnings)
        OK = ~any(contains(warnings(:,2), 'Error'));
    else
        warnings(1,:) = {0, 'Info', 0, 'A szimuláció ideálisan lefutott.'};
    end

    all_warnings{end+1,1} = warnings;

    if ~OK
        warning(['Hiba a ', num2str(num_sim), '. szimulációban.']);
    else
        num_good_sim = num_good_sim + 1;
    end
end

% disp(['Sikeres szimuláció. Szimulációk száma: ', num2str(num_sim)]);
disp(['Összes szimuláció: ', num2str(num_sim)]);
disp(['Jó szimulációk: ', num2str(num_good_sim)]);
disp(['A sikeres szimulációk aránya: ', num2str(num_good_sim/num_sim)]);
disp('Az ideálistól eltérő események:');
disp(warnings)