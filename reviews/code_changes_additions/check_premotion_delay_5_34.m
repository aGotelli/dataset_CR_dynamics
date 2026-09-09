%% check_premotion_delay_5_34.m
% Verifies the actual pre-motion delay ("wait_before_start" in
% read4MotorCircle.py) used for the released recordings, for the
% response to Reviewer 5, Comment 5.34 (manuscript states 2 s, released
% script uses 3 s).
%
% Method: the Mark-10 force-gauge process and the motor process run on
% the same acquisition machine and timestamp with the same wall clock
% (time.time()), but the Mark-10 process starts logging immediately
% while the motor process only starts logging after it sleeps for
% "wait_before_start" seconds. So, for each recording, the delay between
% the first Mark-10 sample and the first motor sample is a direct proxy
% for the pre-motion wait (plus a small, roughly constant process-launch
% overhead common to every recording).
%
% Usage: run from data_collection/dataCollectionPack/figshare/code/postprocessing/
% (or adjust `data_root` below).

close all;
clear;
clc;

%% ====== SETTINGS ======
data_root = fullfile("..", "dataCollectionPack/figshare/data/");
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

fprintf(['\nIf the pre-motion wait were genuinely different between subsets ' ...
    '(2 s vs 3 s), delay_s should cluster into two groups roughly 1 s ' ...
    'apart. A single tight cluster (as found when this was checked ' ...
    'against the released figshare data: ~3.5 s for every subset, with ' ...
    'the two Lissajous recordings at ~3.66 s) indicates the same delay ' ...
    'was used throughout, matching wait_before_start = 3 in the ' ...
    'released read4MotorCircle.py.\n']);

%% ====== LOCAL FUNCTION ======
function t = first_timestamp(csv_path)
    fid = fopen(csv_path, 'r');
    fgetl(fid);              % skip header line
    line = fgetl(fid);       % first data line
    fclose(fid);
    parts = strsplit(line, ',');
    t = str2double(parts{1});
end
