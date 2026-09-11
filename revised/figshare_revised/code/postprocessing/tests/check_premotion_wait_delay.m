%% check_premotion_wait_delay.m
% Estimates the actual pre-motion wait used before each recording
% starts moving the robot.
%
% Method: the Mark-10 force-gauge process and the motor process run on
% the same acquisition machine and timestamp with the same wall clock,
% but the Mark-10 process starts logging immediately while the motor
% process only starts logging after waiting some number of seconds. So,
% for each recording, the delay between the first Mark-10 sample and the
% first motor sample is a direct proxy for that pre-motion wait (plus a
% small, roughly constant process-launch overhead common to every
% recording).
%
% Run this script directly; it locates the dataset relative to its own
% file location (see `data_root` below), so MATLAB's current folder does
% not matter.

close all;
clear;
clc;

%% ====== SETTINGS ======

% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
code_folder = fileparts(fileparts(fileparts(mfilename('fullpath'))));
data_root = fullfile(fileparts(code_folder), "data");

subsets   = ["quasi_static", "dynamic_motion", "contact_motion"];
mark10_file = "dataMark10_+x.csv";
motor_file  = "dataMotor.csv";

%% ====== SCAN RECORDINGS ======
names  = strings(0,1);
deltas = [];

for s = subsets
    subset_dir = fullfile(data_root, s);
    if ~isfolder(subset_dir)
        continue
    end
    entries = dir(subset_dir);
    entries = entries([entries.isdir] & ~startsWith({entries.name}, "."));
    for k = 1:numel(entries)
        rec_dir = fullfile(subset_dir, entries(k).name);
        f_mark10 = fullfile(rec_dir, mark10_file);
        f_motor  = fullfile(rec_dir, motor_file);
        if ~isfile(f_mark10) || ~isfile(f_motor)
            continue
        end

        t_mark10 = first_timestamp(f_mark10);
        t_motor  = first_timestamp(f_motor);

        names(end+1,1)  = s + "/" + string(entries(k).name); %#ok<AGROW>
        deltas(end+1,1) = t_motor - t_mark10; %#ok<AGROW>
    end
end

%% ====== REPORT ======
T = table(names, deltas, 'VariableNames', {'recording', 'delay_s'});
disp(T);

fprintf('\nSummary over %d recordings:\n', height(T));
fprintf('  mean   = %.3f s\n', mean(T.delay_s));
fprintf('  std    = %.3f s\n', std(T.delay_s));
fprintf('  min    = %.3f s\n', min(T.delay_s));
fprintf('  max    = %.3f s\n', max(T.delay_s));

fprintf(['\nIf the pre-motion wait genuinely differed between recordings, ' ...
    'delay_s would cluster into separate groups roughly as far apart as ' ...
    'the difference between those wait times. A single tight cluster ' ...
    'indicates the same wait was used throughout.\n']);

%% ====== LOCAL FUNCTION ======
function t = first_timestamp(csv_path)
    % Reads only the timestamp (first column) of a CSV file's first data
    % row, without loading the rest of the file.
    fid = fopen(csv_path, 'r');
    fgetl(fid);              % skip header line
    line = fgetl(fid);       % first data line
    fclose(fid);
    parts = strsplit(line, ',');
    t = str2double(parts{1});
end
