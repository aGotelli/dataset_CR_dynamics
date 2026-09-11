%% audit_sampling_intervals_5_19.m
%
% This script supports the response to Reviewer 5, Comment 5.19.
%
% THE QUESTION THIS SCRIPT ANSWERS
% ---------------------------------
% The processing code (process_data.m and compare_ft_sensors.m) filters
% every sensor signal with a Butterworth low-pass filter. To design that
% filter, it first needs to know the sensor's sampling rate. It
% estimates that rate as:
%
%       Fs = 1 / median(diff(t))
%
% where t is the vector of timestamps for that sensor. In words: take
% the time gap between every pair of consecutive samples, find the
% MIDDLE value of all those gaps (the median), and say the sampling
% period is that middle value.
%
% This is a fine approach IF the sensor's timestamps are roughly evenly
% spaced (with maybe the occasional stray glitch). It is NOT fine if a
% sensor's timestamps arrive in an uneven pattern, for example several
% samples almost at the same instant, then a pause, then several more
% almost at the same instant again. In that situation the "middle" gap
% can be very different from the "true, on average" gap, so the
% estimated Fs comes out wrong, and the filter ends up doing something
% different from what was intended.
%
% This script checks, for every sensor and every released recording,
% how different the MEAN time gap is from the MEDIAN time gap. If they
% are close, Fs = 1/median(diff(t)) was a safe choice. If they are far
% apart, it was not, and the ratio between them tells us by how much the
% estimated Fs (and therefore the filter's real cutoff frequency) was
% wrong.
%
% WHAT THE SCRIPT PRODUCES
% -------------------------
% 1. A table with one row per (recording, sensor) pair -- this is the
%    detailed, nothing-hidden version, so every single number that goes
%    into the summary below can be traced back to one specific file.
% 2. A table with one row per SENSOR, pooling together the time gaps
%    from every recording that sensor appears in. This is the short
%    table meant to go into the manuscript response.
%
% HOW TO RUN IT
% --------------
% Just run this script -- it locates the dataset relative to its own
% file location (see `data_root` below), so it does not matter what
% MATLAB's current folder happens to be when you press Run. No
% toolboxes beyond base MATLAB are required.

close all;
clear;
clc;


%% ====================================================================
%%  SETTINGS
%% ====================================================================

% Folder that directly CONTAINS quasi_static/, dynamic_motion/ and
% contact_motion/.
%
% We build this path starting from THIS SCRIPT'S OWN location on disk
% (mfilename('fullpath')), instead of from a path like "../../..." that
% is relative to MATLAB's "current folder" setting. A current-folder
% relative path only resolves correctly if you happen to have launched
% MATLAB from exactly the right directory -- when that assumption is
% wrong, fullfile(...) still builds a string, isfolder(...) on it comes
% back false, every "if isfile(...)" check below silently finds
% nothing, and the accumulators stay empty with no visible error until
% the summary table is built at the very end. Building the path from
% the script's own location avoids that failure mode entirely.
%
% This script lives at:
%   reviews/code_changes_additions/audit_sampling_intervals_5_19.m
% and the dataset copy we audit lives at:
%   revised/figshare_revised/data/
% both measured from the repository root, so we go up two folders from
% this script (out of code_changes_additions/, out of reviews/) to
% reach the repository root, then back down into revised/figshare_revised.
this_script_folder = fileparts(mfilename('fullpath'));
reviews_folder      = fileparts(this_script_folder);
repository_root     = fileparts(reviews_folder);

data_root = fullfile(repository_root, "revised", "figshare_revised", "data");

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
%%  counter of how many DIFFERENT RECORDINGS contributed to that list.
%%  Everything starts empty/zero and is filled in as we scan the
%%  dataset below.
%% ====================================================================

ati_gaps_seconds        = [];   ati_recording_count       = 0;
fbgs_gaps_seconds       = [];   fbgs_recording_count      = 0;
optitrack_gaps_seconds  = [];   optitrack_recording_count = 0;
motor_gaps_seconds      = [];   motor_recording_count     = 0;
mark10_gaps_seconds     = [];   mark10_recording_count    = 0;
resense_gaps_seconds    = [];   resense_recording_count   = 0;

% This table collects the detailed, one-row-per-recording-per-sensor
% results, so every number in the final summary can be traced back to a
% specific file. It starts as an empty table and grows by one row every
% time we successfully process a sensor file, using the helper function
% make_detail_row (defined at the bottom of this script).
detail_table = table();


%% ====================================================================
%%  SCAN EVERY RECORDING IN EVERY SUBSET
%% ====================================================================

for subset_index = 1:numel(subset_names)

    subset_name = subset_names{subset_index};
    subset_folder = fullfile(data_root, subset_name);

    if ~isfolder(subset_folder)
        % This subset is not present at data_root -- skip it and keep
        % going, rather than stopping the whole script.
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
        % There are four separate files here, one per tendon direction
        % (+x, +y, -x, -y). We pool the time gaps from all four into
        % the same running list, since they are four copies of the same
        % sensor model and behave the same way -- but a recording
        % should only be counted ONCE towards mark10_recording_count,
        % not once per gauge, so we track that with a simple flag.
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
        % This sensor is only present in the contact_motion recordings.
        % isfile() will simply be false for every quasi_static and
        % dynamic_motion recording, so this block quietly does nothing
        % there -- no special-casing needed.
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
%%  PART 2 OF THE OUTPUT: the short, per-sensor summary table
%%
%%  This is the table meant to be quoted in the manuscript response.
%%  Each row pools EVERY time gap collected for that sensor, across
%%  every recording it appears in (see the accumulator lists built
%%  above), and reports the mean, median, standard deviation, minimum,
%%  maximum, and the mean/median ratio, all in milliseconds.
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
fprintf("(this is the table for the manuscript response)\n");
fprintf("====================================================================\n");
disp(summary_table);


%% ====================================================================
%%  SAVE BOTH TABLES TO CSV
%% ====================================================================

output_folder = "figures";
if ~isfolder(output_folder)
    mkdir(output_folder);
end

detail_output_file  = fullfile(output_folder, "sampling_interval_audit_5_19.csv");
summary_output_file = fullfile(output_folder, "sensor_dt_summary_5_19.csv");

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
    % Reads ONLY the first column of a sensor's raw CSV file. Every raw
    % sensor file in this dataset has its timestamp (Unix epoch
    % seconds) as the first column, whatever the rest of the file
    % contains and however that first column happens to be labeled
    % ("timestamp", "timestamp (s)", etc.) -- so we do not need to know
    % the exact header text, only that it is column number 1.
    import_options = detectImportOptions(csv_file_path);
    import_options.SelectedVariableNames = import_options.VariableNames(1);
    data_table = readtable(csv_file_path, import_options);
    t = data_table.(1);
end


function one_row = make_detail_row(subset_name, recording_name, sensor_label, gaps_in_seconds)
    % Builds one row of the DETAILED table: the statistics for one
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
    % Builds one row of the SUMMARY table: the statistics for one
    % sensor, pooled across every recording it appears in.
    %
    % Why we pool the raw time gaps together, instead of averaging each
    % recording's own mean or median: if we averaged 29 separate
    % "median gap" values together, that average would itself be a kind
    % of median-of-medians, and could hide the same kind of distortion
    % we are trying to check for. Pooling the actual numbers first and
    % only THEN computing one mean/median/etc. avoids that problem and
    % gives one honest, sensor-level statistic.

    % Guard against a sensor that never matched any file (for example
    % because of a folder-layout mismatch upstream). min([]) and
    % max([]) silently return an EMPTY array in MATLAB, not NaN or 0 --
    % if we let that reach the table() call below, it fails with a
    % confusing "All table variables must have the same number of
    % rows" error that does not say which sensor caused it. Checking
    % here instead gives a direct, sensor-specific explanation.
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

    % This ratio is exactly the factor by which the old
    % Fs = 1/median(diff(t)) estimate was wrong for this sensor. A
    % ratio close to 1 means the estimate was fine. A ratio far above 1
    % means the median sat inside a cluster of unusually SHORT gaps
    % (so Fs came out too HIGH, and the filter's real cutoff ended up
    % too LOW). A ratio far below 1 means the opposite.
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
