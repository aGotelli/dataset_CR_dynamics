%% check_sensor_sampling_intervals.m
%
% Checks how uniform each sensor's sampling interval actually is across
% the dataset.
%
% Sampling rate for each sensor is normally estimated as
% Fs = 1 / median(diff(t)), where t is that sensor's timestamp vector.
% This estimate is only accurate if the timestamps are close to evenly
% spaced. If they instead arrive in bursts (several samples close
% together, then a pause, repeat), the median gap can differ
% substantially from the mean gap, and the Fs estimate -- and any
% filter cutoff derived from it -- will be off.
%
% This script computes, for every sensor and every recording, the mean
% and median time gap between consecutive samples, and their ratio
% (mean/median). A ratio near 1 means the sampling was close to
% uniform; a ratio far from 1 flags a sensor/recording where the
% median-based Fs estimate is unreliable.
%
% OUTPUT
% ------
% 1. A detailed table with one row per (recording, sensor) pair.
% 2. A summary table with one row per sensor, pooling the time gaps
%    from every recording that sensor appears in.
%
% Run this script directly; it locates the dataset relative to its own
% file location (see `data_root` below), so MATLAB's current folder
% does not matter. No toolboxes beyond base MATLAB are required.

close all;
clear;
clc;


%% ====================================================================
%%  SETTINGS
%% ====================================================================

% Folder that directly CONTAINS quasi_static/, dynamic_motion/ and
% contact_motion/.
%
% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
code_folder = fileparts(fileparts(fileparts(mfilename('fullpath'))));
data_root = fullfile(fileparts(code_folder), "data");

% If this is not where you keep the dataset on your machine, replace
% the line above with a direct path instead, for example:
%   data_root = "C:\Users\you\dataset_CR_dynamics\revised\figshare_revised\data";

if ~isfolder(data_root)
    error(['Cannot find the dataset folder at:' newline ...
        char(data_root) newline ...
        'This should be the folder that directly contains ' ...
        '"quasi_static", "dynamic_motion" and "contact_motion". ' ...
        'Edit data_root above to point at it directly.']);
end

subset_names = {"quasi_static", "dynamic_motion", "contact_motion"};


%% ====================================================================
%%  ACCUMULATORS
%%
%%  One growing list of time gaps per sensor (in seconds), plus one
%%  counter of how many different recordings contributed to that list.
%% ====================================================================

ati_gaps_seconds        = [];   ati_recording_count       = 0;
fbgs_gaps_seconds       = [];   fbgs_recording_count      = 0;
optitrack_gaps_seconds  = [];   optitrack_recording_count = 0;
motor_gaps_seconds      = [];   motor_recording_count     = 0;
mark10_gaps_seconds     = [];   mark10_recording_count    = 0;
resense_gaps_seconds    = [];   resense_recording_count   = 0;

% Detailed, one-row-per-recording-per-sensor results. Starts empty and
% grows by one row every time a sensor file is processed, via the
% helper function make_detail_row (defined at the bottom of this file).
detail_table = table();


%% ====================================================================
%%  SCAN EVERY RECORDING IN EVERY SUBSET
%% ====================================================================

for subset_index = 1:numel(subset_names)

    subset_name = subset_names{subset_index};
    subset_folder = fullfile(data_root, subset_name);

    if ~isfolder(subset_folder)
        % Subset not present at data_root -- skip it and continue.
        continue
    end

    folder_contents = dir(subset_folder);
    is_a_real_recording_folder = [folder_contents.isdir] & ~startsWith({folder_contents.name}, ".");
    recording_folders = folder_contents(is_a_real_recording_folder);

    for recording_index = 1:numel(recording_folders)

        recording_name = recording_folders(recording_index).name;
        recording_folder = fullfile(subset_folder, recording_name);

        %% ---- ATI force/torque sensor ----
        ati_file = fullfile(recording_folder, "dataATIFT.csv");
        if isfile(ati_file)
            t = read_timestamp_column(ati_file);
            if numel(t) >= 3
                gaps = diff(t);
                ati_gaps_seconds = [ati_gaps_seconds; gaps];
                ati_recording_count = ati_recording_count + 1;
                detail_table = [detail_table; ...
                    make_detail_row(subset_name, recording_name, "ATI force/torque", gaps)];
            end
        end

        %% ---- FBG shape sensor ----
        fbgs_file = fullfile(recording_folder, "dataFBGS.csv");
        if isfile(fbgs_file)
            t = read_timestamp_column(fbgs_file);
            if numel(t) >= 3
                gaps = diff(t);
                fbgs_gaps_seconds = [fbgs_gaps_seconds; gaps];
                fbgs_recording_count = fbgs_recording_count + 1;
                detail_table = [detail_table; ...
                    make_detail_row(subset_name, recording_name, "FBG shape sensor", gaps)];
            end
        end

        %% ---- OptiTrack motion capture ----
        optitrack_file = fullfile(recording_folder, "dataOptiTrack.csv");
        if isfile(optitrack_file)
            t = read_timestamp_column(optitrack_file);
            if numel(t) >= 3
                gaps = diff(t);
                optitrack_gaps_seconds = [optitrack_gaps_seconds; gaps];
                optitrack_recording_count = optitrack_recording_count + 1;
                detail_table = [detail_table; ...
                    make_detail_row(subset_name, recording_name, "OptiTrack motion capture", gaps)];
            end
        end

        %% ---- Motor encoders ----
        motor_file = fullfile(recording_folder, "dataMotor.csv");
        if isfile(motor_file)
            t = read_timestamp_column(motor_file);
            if numel(t) >= 3
                gaps = diff(t);
                motor_gaps_seconds = [motor_gaps_seconds; gaps];
                motor_recording_count = motor_recording_count + 1;
                detail_table = [detail_table; ...
                    make_detail_row(subset_name, recording_name, "Motor encoders", gaps)];
            end
        end

        %% ---- Mark-10 tendon tension gauges (4 files per recording) ----
        % Four separate files, one per tendon direction (+x, +y, -x,
        % -y). Their time gaps are pooled into the same running list
        % since they are the same sensor model, but each recording is
        % only counted once towards mark10_recording_count, not once
        % per gauge.
        mark10_file_names = ["dataMark10_+x.csv", "dataMark10_+y.csv", ...
                              "dataMark10_-x.csv", "dataMark10_-y.csv"];
        this_recording_has_a_mark10_file = false;
        for gauge_index = 1:numel(mark10_file_names)
            mark10_file = fullfile(recording_folder, mark10_file_names(gauge_index));
            if isfile(mark10_file)
                t = read_timestamp_column(mark10_file);
                if numel(t) >= 3
                    gaps = diff(t);
                    mark10_gaps_seconds = [mark10_gaps_seconds; gaps];
                    this_recording_has_a_mark10_file = true;
                    detail_table = [detail_table; ...
                        make_detail_row(subset_name, recording_name, ...
                            "Mark-10 (" + mark10_file_names(gauge_index) + ")", gaps)];
                end
            end
        end
        if this_recording_has_a_mark10_file
            mark10_recording_count = mark10_recording_count + 1;
        end

        %% ---- Resense / HEX12 contact sensor ----
        % Only present in the contact_motion recordings; isfile() is
        % simply false elsewhere, so this block does nothing for the
        % other subsets.
        resense_file = fullfile(recording_folder, "dataResenseFT.csv");
        if isfile(resense_file)
            t = read_timestamp_column(resense_file);
            if numel(t) >= 3
                gaps = diff(t);
                resense_gaps_seconds = [resense_gaps_seconds; gaps];
                resense_recording_count = resense_recording_count + 1;
                detail_table = [detail_table; ...
                    make_detail_row(subset_name, recording_name, "Resense/HEX12 (contact)", gaps)];
            end
        end

    end   % recordings within this subset
end   % subsets


%% ====================================================================
%%  PART 1 OF THE OUTPUT: the detailed, one-row-per-file table
%% ====================================================================

fprintf("\n====================================================================\n");
fprintf("DETAILED TABLE -- one row per (recording, sensor) pair\n");
fprintf("====================================================================\n");
disp(detail_table);


%% ====================================================================
%%  PART 2 OF THE OUTPUT: the per-sensor summary table
%%
%%  Each row pools every time gap collected for that sensor across all
%%  recordings it appears in, and reports the mean, median, standard
%%  deviation, minimum, maximum, and mean/median ratio, in
%%  milliseconds.
%% ====================================================================

row_ati       = make_summary_row("ATI force/torque",         ati_gaps_seconds,       ati_recording_count);
row_fbgs      = make_summary_row("FBG shape sensor",         fbgs_gaps_seconds,      fbgs_recording_count);
row_optitrack = make_summary_row("OptiTrack motion capture", optitrack_gaps_seconds, optitrack_recording_count);
row_motor     = make_summary_row("Motor encoders",           motor_gaps_seconds,     motor_recording_count);
row_mark10    = make_summary_row("Mark-10 tendon tension",   mark10_gaps_seconds,    mark10_recording_count);
row_resense   = make_summary_row("Resense/HEX12 (contact)",  resense_gaps_seconds,   resense_recording_count);

summary_table = [row_ati; row_fbgs; row_optitrack; row_motor; row_mark10; row_resense];

fprintf("\n====================================================================\n");
fprintf("SUMMARY TABLE -- one row per sensor, pooled across all recordings\n");
fprintf("====================================================================\n");
disp(summary_table);


%% ====================================================================
%%  SAVE BOTH TABLES TO CSV
%% ====================================================================

output_folder = fullfile(data_root, "postprocess_calibration");
if ~isfolder(output_folder)
    mkdir(output_folder);
end

detail_output_file  = fullfile(output_folder, "sampling_interval_audit.csv");
summary_output_file = fullfile(output_folder, "sensor_dt_summary.csv");

writetable(detail_table,  detail_output_file);
writetable(summary_table, summary_output_file);

fprintf("\nDetailed (per-recording) table written to: %s\n", detail_output_file);
fprintf("Summary (per-sensor) table written to:     %s\n", summary_output_file);


%% ====================================================================
%%  LOCAL FUNCTIONS
%%  (MATLAB requires local functions to be placed at the end of a
%%  script file. Each one does a single, small, named job.)
%% ====================================================================

function t = read_timestamp_column(csv_file_path)
    % Reads only the first column of a sensor's raw CSV file. Every raw
    % sensor file in this dataset has its timestamp (Unix epoch
    % seconds) as the first column, regardless of how that column is
    % labeled, so only its position needs to be known, not its exact
    % header text.
    import_options = detectImportOptions(csv_file_path);
    import_options.SelectedVariableNames = import_options.VariableNames(1);
    data_table = readtable(csv_file_path, import_options);
    t = data_table.(1);
end


function one_row = make_detail_row(subset_name, recording_name, sensor_label, gaps_in_seconds)
    % Builds one row of the detailed table: the statistics for one
    % sensor, in one specific recording.
    gaps_ms = gaps_in_seconds * 1000;

    mean_gap_ms   = mean(gaps_ms);
    median_gap_ms = median(gaps_ms);

    one_row = table( ...
        string(subset_name), ...
        string(recording_name), ...
        string(sensor_label), ...
        numel(gaps_in_seconds), ...
        mean_gap_ms, ...
        median_gap_ms, ...
        std(gaps_ms), ...
        min(gaps_ms), ...
        max(gaps_ms), ...
        mean_gap_ms / median_gap_ms, ...
        'VariableNames', { ...
            'Subset', 'Recording', 'Sensor', 'NumberOfTimeGaps', ...
            'MeanGap_ms', 'MedianGap_ms', 'StdGap_ms', ...
            'MinGap_ms', 'MaxGap_ms', 'MeanOverMedianRatio'});
end


function one_row = make_summary_row(sensor_name, pooled_gaps_in_seconds, number_of_recordings)
    % Builds one row of the summary table: the statistics for one
    % sensor, pooled across every recording it appears in.
    %
    % Gaps are pooled across recordings before computing statistics,
    % rather than averaging each recording's own mean/median, to avoid
    % a median-of-medians effect that would obscure the thing being
    % measured.

    % min([]) and max([]) return an empty array rather than NaN/0, which
    % would otherwise surface later as a confusing table-construction
    % error. Check explicitly here so a missing sensor gives a clear
    % message.
    if isempty(pooled_gaps_in_seconds)
        error(['No time gaps were collected for sensor "%s". This ' ...
            'means the scan above never found a matching file for ' ...
            'this sensor in any recording -- check that data_root ' ...
            'points at the right folder, and that this sensor''s ' ...
            'CSV file name is spelled the way this script expects.'], ...
            sensor_name);
    end

    gaps_ms = pooled_gaps_in_seconds * 1000;

    mean_gap_ms   = mean(gaps_ms);
    median_gap_ms = median(gaps_ms);

    % Ratio of mean to median gap: close to 1 means Fs = 1/median(diff(t))
    % is a good estimate for this sensor. Above 1 means the median sits
    % inside a cluster of short gaps (Fs overestimated, filter cutoff
    % too low); below 1 means the opposite.
    ratio = mean_gap_ms / median_gap_ms;

    one_row = table( ...
        string(sensor_name), ...
        number_of_recordings, ...
        numel(pooled_gaps_in_seconds), ...
        mean_gap_ms, ...
        median_gap_ms, ...
        std(gaps_ms), ...
        min(gaps_ms), ...
        max(gaps_ms), ...
        ratio, ...
        'VariableNames', { ...
            'Sensor', 'NumberOfRecordings', 'NumberOfTimeGaps', ...
            'MeanGap_ms', 'MedianGap_ms', 'StdGap_ms', ...
            'MinGap_ms', 'MaxGap_ms', 'MeanOverMedianRatio'});
end
