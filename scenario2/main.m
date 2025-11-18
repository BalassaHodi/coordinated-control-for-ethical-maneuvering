% SCENARIO 2

%{
This is the main script to control the different simulations and evaluate them.
%}

clear all;
clear global;
clear;
clc;

global sub_OK dom_OK;
global dom_warnings sub_warnings;
global dom_sebesseg sub_sebesseg;
global dom_kormanyszog sub_kormanyszog;
global vehstate;
global pedestrian;
global dom_vmax sub_vmax;

sub_OK = false;
dom_OK = false;
num_sim = 0;
num_good_sim = 0;
simulations = 20;

warnings = {};

all_warnings = {};
all_sebesseg = {};
all_kormanyszog = {};
all_vehstate = {};
all_subvmax = {};
all_domvmax = {};
all_pedestrian = {};

% FOR RANDOMIZATION - the first elements are the default values
dom_vmax_range = [60, 50, 40, 70];
sub_vmax_range = [40, 50, 60, 70];
pedestrian_y_range = [2.5, 3, 3.5, 4];


rng("shuffle");

% for i=1:simulations
while ~sub_OK || ~dom_OK
    num_sim = num_sim + 1;

    % Randomize initial conditions
    dom_vmax = dom_vmax_range(randi(length(dom_vmax_range)));
    sub_vmax = sub_vmax_range(randi(length(sub_vmax_range)));
    dist = ceil((sub_vmax/3.6)*(24/(dom_vmax/3.6)));
    dom_goal_x = 4+dist+5;
    ped_xmin = dom_goal_x - 15;
    ped_xmax = dom_goal_x -10;
    pedestrian_x_range = ped_xmin:1:ped_xmax;
    pedestrian = [pedestrian_x_range(randi(length(pedestrian_x_range))), pedestrian_y_range(randi(length(pedestrian_y_range)))];

    disp(['Simulation ', num2str(num_sim), ' initial values:']);
    disp(['dom_vmax = ' num2str(dom_vmax)]);
    disp(['sub_vmax = ' num2str(sub_vmax)]);
    disp(['pedestrian = [' num2str(pedestrian(1)), ', ' num2str(pedestrian(2)), ']']);
    disp(['Simulation ', num2str(num_sim), ' is running...']);
    run Simulation.m

    % If the warnings are empty
    if isempty(dom_warnings)
        dom_warnings(1,:) = {'DOM', 0, 'Info', 0, 'A szimulációban minden ideálisan lefutott.'};
    end
    if isempty(sub_warnings)
        sub_warnings(1,:) = {'SUB', 0, 'Info', 0, 'A szimulációban minden ideálisan lefutott.'};
    end

    % Create global warnings
    % If errors
    if any(contains(dom_warnings(:,3), 'Error')) && ~any(contains(sub_warnings(:,3), 'Error'))
        warnings(end+1,:) = {1, 'Error', num_sim, 'Vészhelyzet! A domináns jármű vészhelzetben volt!'};
    elseif any(contains(sub_warnings(:,3), 'Error')) && ~any(contains(dom_warnings(:,3), 'Error'))
        warnings(end+1,:) = {2, 'Error', num_sim, 'Vészhelyzet! Az alárendelt jármű vészhelyzetben volt!'};
    elseif any(contains(dom_warnings(:,3), 'Error')) && any(contains(sub_warnings(:,3), 'Error'))
        warnings(end+1,:) = {3, 'Error', num_sim, 'Vészhelyzet! Mindkét jármű vészhelyzetben volt!'};

    % If no error, just warnings
    elseif any(contains(dom_warnings(:,3), 'Warning')) || any(contains(sub_warnings(:,3), 'Warning'))
        warnings(end+1,:) = {4, 'Warning', num_sim, 'A szimulációban legfeljebb csak "warning" típusú üzenetek voltak.'};

    % If no warnings just infos
    elseif any(contains(dom_warnings(:,3), 'Info')) || any(contains(sub_warnings(:,3), 'Info'))
        warnings(end+1,:) = {5, 'Info', num_sim, 'A szimulációban csak "info" típusú üzenetek voltak.'};

    % If none of them
    else
        warnings(end+1,:) = {0, 'Info', num_sim, 'A szimuláció ideálisan lefutott.'};
    end

    
    % Store all the simulation data
    all_warnings{end+1,1} = warnings; all_warnings{end,2} = dom_warnings; all_warnings{end,3} = sub_warnings;
    all_sebesseg{end+1,1} = dom_sebesseg; all_sebesseg{end,2} =  sub_sebesseg;
    all_kormanyszog{end+1,1} = dom_kormanyszog; all_kormanyszog{end,2} =  sub_kormanyszog;
    all_vehstate{end+1,1} = vehstate;
    all_domvmax{end+1,1} = dom_vmax;
    all_subvmax{end+1,1} = sub_vmax;
    all_pedestrian{end+1,1} = pedestrian;


    if ~sub_OK || ~dom_OK
        warning(['Hiba a ', num2str(num_sim), '. szimulációban.']);
    else
        num_good_sim = num_good_sim + 1;
        disp('A szimuláció sikeresen lefutott.');
    end
    disp(' ');
end

% disp(['Sikeres szimuláció. Szimulációk száma: ', num2str(num_sim)]);
disp(['Összes szimuláció: ', num2str(num_sim)]);
disp(['Jó szimulációk: ', num2str(num_good_sim)]);
disp(['A sikeres szimulációk aránya: ', num2str(num_good_sim/num_sim)]);

% Save all the simulation results
% save scenario2_random_init_sim_results1.mat all_domvmax all_kormanyszog all_pedestrian all_sebesseg all_subvmax all_vehstate all_warnings num_sim num_good_sim;