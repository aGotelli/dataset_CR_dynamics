%% check_optitrack_validity_5_28.m
%
% This script supports the response to Reviewer 5, Comment 5.28.
%
% THE QUESTION THIS SCRIPT ANSWERS
% ---------------------------------
% Every raw OptiTrack file (dataOptiTrack.csv) contains, for every disk and
% every sample, a "<disk>_is_valid" flag: Motive (the OptiTrack software)
% sets this to 0 whenever it could not reconstruct that disk's pose for
% that sample, typically because too many of its markers were occluded
% from the cameras. The reviewer pointed out that data_optitrack.m loads
% this flag but never actually checks it, and that the released, processed
% data does not say anywhere whether any frames were ever occluded.
%
% This script answers that question directly: it calls data_optitrack.m
% on every released recording, exactly the way process_data.m does, and
% counts how many valid and invalid frames come back for every disk of
% every recording. If a disk was ever occluded, this script also reports
% exactly which recording, which disk, and which sample -- so the finding
% can be double-checked by hand.
%
% WHAT THE SCRIPT PRODUCES
% -------------------------
% 1. A table with one row per (recording, disk) pair, giving the number of
%    samples, how many were valid, and how many were invalid for that
%    disk in that recording. Saved as detailed_optitrack_validity_5_28.csv.
% 2. If (and only if) any invalid frames are found anywhere, a second
%    table listing every single invalid sample individually (recording,
%    disk, sample index, and timestamp), so it can be located and
%    inspected. Saved as invalid_optitrack_frames_5_28.csv.
% 3. A short, unambiguous printed summary stating exactly how many
%    recordings, disks, and individual samples were checked, and how many
%    of those samples were invalid -- this is the sentence meant to be
%    quoted directly in the manuscript and the response letter.
%
% HOW TO RUN IT
% --------------
% Just run this script -- it locates the dataset and the data_optitrack.m
% function relative to its own file location (see the SETTINGS section
% below), so it does not matter what MATLAB's current folder happens to
% be when you press Run. No toolboxes beyond base MATLAB are required.

close all;
clear;
clc;


%% ====================================================================
%%  SETTINGS
%% ====================================================================

% This script lives at:
%   reviews/code_changes_additions/check_optitrack_validity_5_28.m
% We use its own location on disk (mfilename('fullpath')), rather than a
% path relative to MATLAB's "current folder" setting, to find both the
% dataset and the postprocessing code -- a current-folder-relative path
% only works if MATLAB happens to be pointed at exactly the right
% directory when you press Run, and silently finding nothing is exactly
% the kind of bug this project has already run into once before with a
% path written this way (see audit_sampling_intervals_5_19.m).
this_script_folder = fileparts(mfilename('fullpath'));
reviews_folder      = fileparts(this_script_folder);
repository_root     = fileparts(reviews_folder);

% Folder that directly CONTAINS quasi_static/, dynamic_motion/ and
% contact_motion/ -- the same dataset copy used for the Comment 5.19
% audit script, for consistency.
data_root = fullfile(repository_root, "revised", "figshare_revised", "data");

if ~isfolder(data_root)
    error(['Cannot find the dataset folder at:' newline ...
        char(data_root) newline ...
        'This should be the folder that directly contains ' ...
        '"quasi_static", "dynamic_motion" and "contact_motion". ' ...
        'Edit data_root above to point at it directly.']);
end

% Folder that contains data_optitrack.m, so we can call it. This must be
% the SAME postprocessing/ tree that produced the released dataset in
% revised/figshare_revised/data (the one data_root, above, points at) --
% not the older working copy under data_collection/dataCollectionPack/,
% which is a separate, out-of-date snapshot of this code.
outils_folder = fullfile(repository_root, "revised", ...
    "figshare_revised", "code", "postprocessing", "outils");

if ~isfolder(outils_folder)
    error(['Cannot find the outils/ folder (which should contain ' ...
        'data_optitrack.m) at:' newline char(outils_folder) newline ...
        'Edit outils_folder above to point at it directly.']);
end

addpath(outils_folder);

subset_names = {"quasi_static", "dynamic_motion", "contact_motion"};


%% ====================================================================
%%  ACCUMULATORS
%%
%%  detail_table collects one row per (recording, disk) pair, so every
%%  number in the final summary can be traced back to a specific disk in
%%  a specific recording. invalid_frame_table collects one row per
%%  INDIVIDUAL invalid sample, if any are ever found. Both start as empty
%%  tables and grow via the helper functions defined at the bottom of
%%  this script.
%% ====================================================================

detail_table = table();
invalid_frame_table = table();

% Running totals across the whole dataset. These are what the final
% printed summary and the manuscript sentence are built from.
total_recordings_checked          = 0;
total_disk_recording_pairs_checked = 0;
total_frame_entries_checked        = 0;
total_invalid_frame_entries_found  = 0;


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

        optitrack_file = fullfile(recording_folder, "dataOptiTrack.csv");
        if ~isfile(optitrack_file)
            % This recording has no OptiTrack file at all -- nothing to
            % check here, move on to the next recording.
            continue
        end

        % use_resense controls whether data_optitrack.m expects a sixth
        % disk (the Resense/HEX12 wand, used only in contact recordings).
        % We determine this the same way process_data.m's author intended
        % it to be determined -- by whether this specific recording
        % actually has a dataResenseFT.csv file -- rather than assuming it
        % from the subset name, since not every contact_motion recording
        % necessarily has one.
        resense_file = fullfile(recording_folder, "dataResenseFT.csv");
        use_resense_this_recording = isfile(resense_file);

        % Call data_optitrack.m exactly the way process_data.m does,
        % except we also ask for its sixth, optional output:
        % is_valid_disk, the per-disk, per-sample validity flag.
        [N_disks, timestamps, poses_disks, rel_poses_disks, ...
            rel_kinematics_disks, is_valid_disk] = ...
            data_optitrack(optitrack_file, use_resense_this_recording);

        total_recordings_checked = total_recordings_checked + 1;
        n_time_this_recording = size(is_valid_disk, 1);

        for disk_index = 1:N_disks

            % The CSV columns are named disk_0, disk_1, ... (0-indexed),
            % while this MATLAB loop counts disk_index from 1. disk_number
            % below converts back to the 0-indexed name so the reported
            % disk number matches what you would see if you opened the
            % CSV file yourself.
            disk_number = disk_index - 1;

            is_valid_this_disk = is_valid_disk(:, disk_index);

            n_valid_this_disk   = sum(is_valid_this_disk == 1);
            n_invalid_this_disk = sum(is_valid_this_disk == 0);

            total_disk_recording_pairs_checked = total_disk_recording_pairs_checked + 1;
            total_frame_entries_checked = total_frame_entries_checked + n_time_this_recording;
            total_invalid_frame_entries_found = total_invalid_frame_entries_found + n_invalid_this_disk;

            detail_table = [detail_table; ...
                make_detail_row(subset_name, recording_name, disk_number, ...
                    n_time_this_recording, n_valid_this_disk, n_invalid_this_disk)];

            if n_invalid_this_disk > 0
                % At least one frame was occluded for this disk in this
                % recording. Record exactly which samples, so this can be
                % located and inspected by hand rather than just counted.
                invalid_sample_indices = find(is_valid_this_disk == 0);
                invalid_sample_timestamps = timestamps(invalid_sample_indices);

                for k = 1:numel(invalid_sample_indices)
                    invalid_frame_table = [invalid_frame_table; ...
                        make_invalid_frame_row(subset_name, recording_name, ...
                            disk_number, invalid_sample_indices(k), ...
                            invalid_sample_timestamps(k))];
                end

                fprintf(['INVALID FRAME(S) FOUND -- subset "%s", recording "%s", ' ...
                    'disk %d: %d of %d samples invalid.\n'], ...
                    subset_name, recording_name, disk_number, ...
                    n_invalid_this_disk, n_time_this_recording);
            end

        end

    end

end


%% ====================================================================
%%  SAVE THE DETAILED TABLE
%% ====================================================================

output_folder = fullfile(this_script_folder, "figures");
if ~isfolder(output_folder)
    mkdir(output_folder);
end

detail_csv_path = fullfile(output_folder, "detailed_optitrack_validity_5_28.csv");
writetable(detail_table, detail_csv_path);

fprintf("\nDetailed (recording x disk) validity table written to:\n  %s\n", detail_csv_path);


%% ====================================================================
%%  SAVE THE LIST OF INVALID FRAMES, IF ANY WERE FOUND
%% ====================================================================

if height(invalid_frame_table) > 0
    invalid_csv_path = fullfile(output_folder, "invalid_optitrack_frames_5_28.csv");
    writetable(invalid_frame_table, invalid_csv_path);
    fprintf("List of every individual invalid frame written to:\n  %s\n", invalid_csv_path);
else
    fprintf("No invalid frames were found anywhere, so no invalid-frame list was written.\n");
end


%% ====================================================================
%%  FINAL SUMMARY -- this is the statement to quote in the manuscript
%%  and the response letter.
%% ====================================================================

fprintf("\n====================================================================\n");
fprintf("SUMMARY\n");
fprintf("====================================================================\n");
fprintf("Recordings checked:                 %d\n", total_recordings_checked);
fprintf("(recording, disk) pairs checked:    %d\n", total_disk_recording_pairs_checked);
fprintf("Individual frame-validity entries checked: %d\n", total_frame_entries_checked);
fprintf("Invalid (occluded) frame entries found:    %d\n", total_invalid_frame_entries_found);

if total_invalid_frame_entries_found == 0
    fprintf(['\nAll %d recordings, covering %d individual OptiTrack disk-frame ' ...
        'validity entries, are fully valid -- Motive never reported an ' ...
        'occluded/invalid disk pose anywhere in the released dataset.\n'], ...
        total_recordings_checked, total_frame_entries_checked);
else
    fprintf(['\n%d of %d OptiTrack disk-frame entries were invalid. See ' ...
        'invalid_optitrack_frames_5_28.csv for the full list of exactly ' ...
        'which recording, disk and sample each one is.\n'], ...
        total_invalid_frame_entries_found, total_frame_entries_checked);
end


%% ====================================================================
%%  LOCAL FUNCTIONS
%% ====================================================================

function one_row = make_detail_row(subset_name, recording_name, disk_number, ...
        n_time, n_valid, n_invalid)
    % Builds one row of the DETAILED table: for one disk, in one specific
    % recording, how many samples were valid vs. invalid.
    one_row = table( ...
        string(subset_name), ...
        string(recording_name), ...
        disk_number, ...
        n_time, ...
        n_valid, ...
        n_invalid, ...
        'VariableNames', { ...
            'Subset', 'Recording', 'DiskNumber', ...
            'NumberOfSamples', 'NumberOfValidSamples', 'NumberOfInvalidSamples'});
end


function one_row = make_invalid_frame_row(subset_name, recording_name, ...
        disk_number, sample_index, sample_timestamp)
    % Builds one row identifying a SINGLE invalid (occluded) sample: which
    % recording, which disk, which row of the raw CSV file (sample_index),
    % and the timestamp Motive attached to it. Kept as one row per invalid
    % sample, rather than only a count, so that if this ever finds
    % something, it can be opened and looked at directly rather than
    % taken on faith.
    one_row = table( ...
        string(subset_name), ...
        string(recording_name), ...
        disk_number, ...
        sample_index, ...
        sample_timestamp, ...
        'VariableNames', { ...
            'Subset', 'Recording', 'DiskNumber', ...
            'SampleIndex', 'Timestamp_s'});
end
