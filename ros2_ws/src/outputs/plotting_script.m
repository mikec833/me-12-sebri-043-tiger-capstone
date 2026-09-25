baseFolder = "C:\Users\Michael's PC\OneDrive - The University of Melbourne\Uni\Master's\Sem 2 2026\Capstone\robot_test_24_09";
runID = "20260924_191935";
xVar = 'time_s';
yVar = 'yaw';

allFiles = dir(fullfile(baseFolder, '**', '*.csv'));
allFiles = allFiles(~[allFiles.isdir]);

matchedFiles = {};
for i = 1:length(allFiles)
    fileName = allFiles(i).name;
    if contains(fileName, runID)
        matchedFiles{end+1} = fullfile(allFiles(i).folder, fileName);
    end
end

fprintf('Matched %d files for run %s\n', length(matchedFiles), runID);
for i = 1:length(matchedFiles)
    T = readtable(matchedFiles{i});
    fprintf('%s\n  rows = %d\n  vars = %s\n', ...
        matchedFiles{i}, ...
        height(T), ...
        strjoin(T.Properties.VariableNames, ', '));
end

% Convert full paths to folder paths one at a time
folderPaths = cellfun(@fileparts, matchedFiles, 'UniformOutput', false);
sensorFolders = unique(folderPaths);

figure('Color','w');

plotCount = 0;
for f = 1:length(sensorFolders)
    folderPath = sensorFolders{f};
    filesInFolder = dir(fullfile(folderPath, '*.csv'));

    hasPlotData = false;

    for i = 1:length(filesInFolder)
        fileName = filesInFolder(i).name;
        if ~contains(fileName, runID)
            continue;
        end

        filePath = fullfile(filesInFolder(i).folder, fileName);
        T = readtable(filePath);

        if isempty(T) || height(T) == 0
            continue;
        end

        if ismember(xVar, T.Properties.VariableNames) && ismember(yVar, T.Properties.VariableNames)
            hasPlotData = true;
            break;
        end
    end

    if ~hasPlotData
        continue;
    end

    plotCount = plotCount + 1;
    subplot(1, plotCount, plotCount);
    hold on; grid on;

    for i = 1:length(filesInFolder)
        fileName = filesInFolder(i).name;
        if ~contains(fileName, runID)
            continue;
        end

        filePath = fullfile(filesInFolder(i).folder, fileName);
        T = readtable(filePath);

        if isempty(T) || height(T) == 0
            continue;
        end

        if ismember(xVar, T.Properties.VariableNames) && ismember(yVar, T.Properties.VariableNames)
            plot(T.(xVar), T.(yVar), 'LineWidth', 1.5, 'DisplayName', fileName);
        end
    end

    folderTitle = strrep(folderPath, baseFolder, '');
    title(folderTitle, 'Interpreter', 'none');
    xlabel(xVar);
    ylabel(yVar);
end

if plotCount == 0
    warning('No valid run data found for %s', runID);
else
    legend('show', 'Location', 'bestoutside');
end