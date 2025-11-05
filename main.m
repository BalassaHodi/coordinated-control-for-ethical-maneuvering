%{
This is the main script to control the different simulations and evaluate them.
%}

clc;
global OK;

OK = false;
num_sim = 0;
num_good_sim = 0;

% for i=1:20
while ~OK
    num_sim = num_sim + 1;
    run Simulation.m
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