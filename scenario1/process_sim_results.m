clc;
clear;
close all;

% READ .MAT FILES
% File pattern and variable names
files = arrayfun(@(k) sprintf('random_init_sim_results%d.mat', k), 1:10, 'uni', false);
cellVars = {'all_warnings','all_sebesseg','all_kormanyszog','all_vehstate','all_vhmax','all_vamax','all_pedestrian'};
numVars  = {'num_sim','num_good_sim'};

% Containers for results
merged = struct();
for v = 1:numel(cellVars)
    merged.(cellVars{v}) = {};    % start empty cell column
end
num_sim = nan(numel(files),1);
num_good_sim = nan(numel(files),1);

% Loop and merge
for k = 1:numel(files)
    S = load(files{k});           % loads fields into struct S

    % merge cell-array variables (vertical concat)
    for v = 1:numel(cellVars)
        name = cellVars{v};
        if isfield(S, name)
            % ensure column cell arrays before concatenation
            C = S.(name);
            if isrow(C)
                C = C(:);
            end
            merged.(name) = [merged.(name); C];
        else
            warning('File %s does not contain %s', files{k}, name);
        end
    end

    % collect doubles
    if isfield(S,'num_sim');      num_sim(k) = S.num_sim;        end
    if isfield(S,'num_good_sim'); num_good_sim(k) = S.num_good_sim; end
end

% Now every merged data are in the variable called merged, so get them:
all_warnings = merged.all_warnings;
all_sebesseg = merged.all_sebesseg;
all_kormanyszog = merged.all_kormanyszog;
all_vehstate = merged.all_vehstate;
all_vhmax = merged.all_vhmax;
all_vamax = merged.all_vamax;
all_pedestrian = merged.all_pedestrian;

% Free the workspace from not important variables
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
    if ~any(contains(one_warning(:,2), 'Warning')) && ~any(contains(one_warning(:,2), 'Error'))
        num_info = num_info + 1;
    elseif any(contains(one_warning(:,2), 'Warning')) && ~any(contains(one_warning(:,2), 'Error'))
        % Here is a problem with the code, because every warning with the error code 8
        % are actually info, not warning! So only error code 2 and 5 are true warnings!
        if any(cell2mat(one_warning(:,1)) == 2) || any(cell2mat(one_warning(:,1)) == 5)
            num_warning = num_warning + 1;
        else
            num_info = num_info + 1;
        end
    elseif any(contains(one_warning(:,2), 'Error'))
        num_error = num_error + 1;
    end
end

% Visualize the different outputs in simulations
figure;
pie([num_info, num_warning, num_error], {sprintf('No issue: %.2f%%', (num_info/200)*100), sprintf('Only warnings: %.2f%%', (num_warning/200)*100), sprintf('Errors: %.2f%%', (num_error/200)*100)});
title('Outputs of the simulations');
grid on;

% The types of warnings
startpose_not_good = 0;
no_rrt_use_prev_path = 0;
for i = 1:size(all_warnings,1)
    one_warning = all_warnings{i,1};
    if any(contains(one_warning(:,2), 'Warning')) && ~any(contains(one_warning(:,2), 'Error'))
        % Here is a problem with the code, because every warning with the error code 8
        % are actually info, not warning!
        if any(cell2mat(one_warning(:,1)) == 2) || any(cell2mat(one_warning(:,1)) == 5)
            if any(cell2mat(one_warning(:,1)) == 2)
                startpose_not_good = startpose_not_good + 1;
            end
            if any(cell2mat(one_warning(:,1)) == 5)
                no_rrt_use_prev_path = no_rrt_use_prev_path + 1;
            end
        end
    end
end

% Visualize the different types of warnings
figure;
pie([startpose_not_good, no_rrt_use_prev_path], {sprintf('StartPose not good, previous refPath is used: %d', startpose_not_good), sprintf('No RRT in time step, previous refPath is used: %d', no_rrt_use_prev_path)});
title('Types of warnings');
grid on;

% See the different types of errors
num_critical_error = 0;
num_non_critical_error = 0;
critical_error_indices = [];
non_critical_error_indices = [];
for i = 1:size(all_warnings,1)
    one_warning = all_warnings{i,1};
    if any(contains(one_warning(:,2), 'Error'))
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

% Visualize the different types of errors
figure;
pie([num_critical_error, num_non_critical_error], {sprintf('Critical error (collusion): %d', num_critical_error), sprintf('Non critical errors (no collusion just hard brake): %d', num_non_critical_error)});
title('Types of errors');
grid on;