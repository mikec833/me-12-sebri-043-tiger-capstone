baseFolder = 'C:\Users\Michael''s PC\OneDrive - The University of Melbourne\Uni\Master''s\Sem 2 2026\Capstone\robot_test_24_09';
runID = '20260924_191935';

runFolder = fullfile(baseFolder, ['run_' runID]);
runFolder = char(runFolder);

if ~isfolder(runFolder)
    error('Run folder not found: %s', runFolder);
end

files = dir(fullfile(runFolder, '*.csv'));
files = files(~[files.isdir]);

cmdFile = '';
uwbFile = '';
wheelFile = '';

for i = 1:length(files)
    name = files(i).name;
    if contains(name, 'cmd_vel')
        cmdFile = fullfile(runFolder, name);
    elseif contains(name, 'uwb')
        uwbFile = fullfile(runFolder, name);
    elseif contains(name, 'wheel')
        wheelFile = fullfile(runFolder, name);
    end
end

figure('Color', 'w');

% ---- cmd_vel subplot ----
if ~isempty(cmdFile)
    Tcmd = readtable(cmdFile);
    subplot(3,1,1);
    hold on; grid on;

    if ismember('linear_x', Tcmd.Properties.VariableNames)
        plot(Tcmd.time_s, Tcmd.linear_x, 'b', 'LineWidth', 1.5);
    end
    if ismember('angular_z', Tcmd.Properties.VariableNames)
        plot(Tcmd.time_s, Tcmd.angular_z, 'r', 'LineWidth', 1.5);
    end

    xlabel('time_s', 'Interpreter', 'none');
    ylabel('command', 'Interpreter', 'none');
    title('cmd_vel log', 'Interpreter', 'none');
    legend({'linear_x','angular_z'}, 'Interpreter', 'none', 'Location', 'best');
    ylim([-2 2]);
end

% ---- wheel speed subplot ----
if ~isempty(wheelFile)
    Twheel = readtable(wheelFile);
    subplot(3,1,2);
    hold on; grid on;

    if ismember('ref_left', Twheel.Properties.VariableNames)
        plot(Twheel.time_s, Twheel.ref_left, 'b--', 'LineWidth', 1.5);
    end
    if ismember('ref_right', Twheel.Properties.VariableNames)
        plot(Twheel.time_s, Twheel.ref_right, 'r--', 'LineWidth', 1.5);
    end
    if ismember('meas_left', Twheel.Properties.VariableNames)
        plot(Twheel.time_s, Twheel.meas_left, 'b', 'LineWidth', 1.5);
    end
    if ismember('meas_right', Twheel.Properties.VariableNames)
        plot(Twheel.time_s, Twheel.meas_right, 'r', 'LineWidth', 1.5);
    end

    xlabel('time_s', 'Interpreter', 'none');
    ylabel('wheel speed (rad/s)', 'Interpreter', 'none');
    title('wheel_speed log', 'Interpreter', 'none');
    legend({'ref_left','ref_right','meas_left','meas_right'}, ...
           'Interpreter', 'none', 'Location', 'best');
    ylim([-8 8]);
end

% ---- UWB subplot ----
if ~isempty(uwbFile)
    Tuwb = readtable(uwbFile);
    subplot(3,1,3);
    hold on; grid on;

    if ismember('x', Tuwb.Properties.VariableNames)
        plot(Tuwb.time_s, Tuwb.x, 'k', 'LineWidth', 1.5);
    end
    if ismember('y', Tuwb.Properties.VariableNames)
        plot(Tuwb.time_s, Tuwb.y, 'b', 'LineWidth', 1.5);
    end
    if ismember('z', Tuwb.Properties.VariableNames)
        plot(Tuwb.time_s, Tuwb.z, 'r', 'LineWidth', 1.5);
    end

    xlabel('time_s', 'Interpreter', 'none');
    ylabel('position (m)', 'Interpreter', 'none');
    title('uwb log', 'Interpreter', 'none');
    legend({'x','y','z'}, 'Interpreter', 'none', 'Location', 'best');
end