clc;
clear all;
close all;

%{
sum_succes_rate = 0;
for i = 1:10
    filename = sprintf('scenario2_random_init_sim_results%d.mat', i);
    load(filename);

    sim = i;
    success_rate = num_good_sim/num_sim;
    sum_succes_rate = sum_succes_rate + success_rate;
    disp(['Simulation: ' num2str(sim), '. succes rate: ' num2str(success_rate)]);
    clearvars -except sum_succes_rate;
end

average_success_rate = sum_succes_rate / 10;
disp(['Average success rate over simulations: ' num2str(average_success_rate)]);
%}


% LOAD FILES
files = fullfile("scenario2_random_init_sim_results%d.mat");
N = 10;

% Initialize empty masters
all_domvmax = {};
all_kormanyszog = {};
all_pedestrian = {};
all_sebesseg = {};
all_subvmax = {};
all_vehstate = {};
all_warnings = {};
num_sim = [];
num_good_sim = [];

for k = 1:N
    S = load(sprintf(files, k));         % load into struct S

    % Append cell arrays (ensure column-wise concatenation)
    if isfield(S,'all_domvmax') && ~isempty(S.all_domvmax)
        all_domvmax = [all_domvmax; S.all_domvmax]; % Keep the same number of columns
    end
    if isfield(S,'all_kormanyszog') && ~isempty(S.all_kormanyszog)
        all_kormanyszog = [all_kormanyszog; S.all_kormanyszog]; % Keep the same number of columns
    end
    if isfield(S,'all_pedestrian') && ~isempty(S.all_pedestrian)
        all_pedestrian = [all_pedestrian; S.all_pedestrian]; % Keep the same number of columns
    end
    if isfield(S,'all_sebesseg') && ~isempty(S.all_sebesseg)
        all_sebesseg = [all_sebesseg; S.all_sebesseg]; % Keep the same number of columns
    end
    if isfield(S,'all_subvmax') && ~isempty(S.all_subvmax)
        all_subvmax = [all_subvmax; S.all_subvmax]; % Keep the same number of columns
    end
    if isfield(S,'all_vehstate') && ~isempty(S.all_vehstate)
        all_vehstate = [all_vehstate; S.all_vehstate]; % Keep the same number of columns
    end
    if isfield(S,'all_warnings') && ~isempty(S.all_warnings)
        all_warnings = [all_warnings; S.all_warnings]; % Keep the same number of columns
    end

    % Append numeric arrays (treat as column vectors)
    if isfield(S,'num_sim') && ~isempty(S.num_sim)
        num_sim = [num_sim; S.num_sim(:)];
    end
    if isfield(S,'num_good_sim') && ~isempty(S.num_good_sim)
        num_good_sim = [num_good_sim; S.num_good_sim(:)];
    end
end
clearvars -except -regexp all_ num_




% PROCESS THE DATA
% Calculate the success rate of the simulations
success_rate = sum(num_good_sim)/sum(num_sim);
disp(['A sikeres szimulációk aránya: ', num2str(success_rate)]);

% Visualize the success rate
figure;
pie([success_rate, 1-success_rate], {sprintf('Successful: %.2f%%', success_rate*100), sprintf('Unsuccessful: %.2f%%', (1-success_rate)*100)});
title('Success Rate of Simulations');
grid on;


% See the different warnings
num_info = 0;
num_warning = 0;
num_error = 0;
for i = 1:size(all_warnings,1)
    one_warning = all_warnings{i,1};
    if contains(one_warning{1,2}, 'Info')
        num_info = num_info + 1;
    elseif contains(one_warning{1,2}, 'Warning')
        num_warning = num_warning + 1;
    elseif contains(one_warning{1,2}, 'Error')
        num_error = num_error + 1;
    end
end

% Visualize the different outputs in simulations
figure;
pie([num_info, num_warning, num_error], {sprintf('No issue: %.2f%%', (num_info/200)*100), sprintf('Only warnings: %.2f%%', (num_warning/200)*100), sprintf('Errors: %.2f%%', (num_error/200)*100)});
title('Outputs of the simulations');
grid on;


% The types of errors
% See the different types of errors
num_dom_error = 0;
num_sub_error = 0;
num_both_error = 0;
for i = 1:size(all_warnings,1)
    one_warning = all_warnings{i,1};
    if contains(one_warning{1,2}, 'Error')
        num_error = num_error + 1;
        if one_warning{1,1} == 1
            num_dom_error = num_dom_error + 1;
        elseif one_warning{1,1} == 2
            num_sub_error = num_sub_error + 1;
        elseif one_warning{1,1} == 3
            num_both_error = num_both_error + 1;
        end
    end
end

% Visualize the different outputs in simulations
% Visualize the different types of errors in a pie chart
figure;
pie([num_dom_error, num_sub_error, num_both_error], {sprintf('Dominant errors: %.2f%%', (num_dom_error/size(all_warnings,1))*100), sprintf('Subordinate errors: %.2f%%', (num_sub_error/size(all_warnings,1))*100), sprintf('Both errors: %.2f%%', (num_both_error/size(all_warnings,1))*100)});
title('Types of Errors in Simulations');
grid on;


% See the dominant errors
num_critical_error = 0;
num_non_critical_error = 0;
critical_error_indices = [];
non_critical_error_indices = [];
for i = 1:size(all_warnings,1)
    one_warning = all_warnings{i,1};
    if contains(one_warning{1,2}, 'Error')
        num_error = num_error + 1;
        if one_warning{1,1} == 1
            one_vehstate = all_vehstate{i,1};
            one_pedestrian = all_pedestrian{i,1};
            was_crit_error = false;
            for j = 1:size(one_vehstate,1)
                if one_vehstate(j,1) >= one_pedestrian(1)-1.5 && one_vehstate(j,1) <= one_pedestrian(1)+1.5 && one_vehstate(j,2) <= one_pedestrian(2)+1.5 && one_vehstate(j,2) >= one_pedestrian(2)-1.5
                    num_critical_error = num_critical_error + 1;
                    critical_error_indices(end+1,1) = i;
                    was_crit_error = true;
                    break
                end
            end
    
            if ~was_crit_error 
                num_non_critical_error = num_non_critical_error + 1;
                non_critical_error_indices(end+1,1) = i;
            end
        end
    end
end

% Visualize the different types of errors
figure;
pie([num_critical_error, num_non_critical_error], {sprintf('Critical error (collusion): %d', num_critical_error), sprintf('Non critical errors (no collusion just hard brake): %d', num_non_critical_error)});
title('Types of errors');
grid on;