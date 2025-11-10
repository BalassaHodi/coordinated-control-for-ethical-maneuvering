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
simulations = 10;
all_warnings = {};

% for i=1:simulations
while ~OK
    num_sim = num_sim + 1;
    run Simulation.m

    OK = ~any(contains(warnings, "Vészhelyzet!"));
    all_warnings{end+1} = warnings;

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
for i = 1:length(warnings)
    disp(warnings{i})
end