%% check_optitrack_frame_validity.m
%
% Checks, for every released recording, whether Motive (the OptiTrack
% software) ever failed to reconstruct a disk's pose for a sample. Every
% raw OptiTrack file (dataOptiTrack.csv) carries a "<disk>_is_valid" flag
% per disk per sample, set to 0 whenever too many of that disk's markers
% were occluded from the cameras; data_optitrack.m loads this flag but
% does not check it.
%
% This script calls data_optitrack.m on every recording, exactly the way
% process_data.m does, and counts how many valid and invalid frames come
% back for every disk of every recording. Whenever a disk was ever
% occluded, it also reports exactly which recording, disk, and sample,
% so the finding can be traced back to the raw file.
%
% OUTPUT
% ------
% 1. A table with one row per (recording, disk) pair: number of samples,
%    and how many were valid/invalid. Saved as
%    detailed_optitrack_validity.csv.
% 2. If any invalid frames are found, a second table listing every
%    invalid sample individually (recording, disk, sample index,
%    timestamp). Saved as invalid_optitrack_frames.csv.
% 3. A printed summary: how many recordings, disks, and individual
%    samples were checked, and how many were invalid.
%
% Run this script directly; it locates the dataset and outils/ relative
% to its own file location, so MATLAB's current folder does not matter.
% No toolboxes beyond base MATLAB are required.

close all;
clear;
clc;


%% ====================================================================
%%  SETTINGS
%% ====================================================================

% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
this_script_folder    = fileparts(mfilename('fullpath'));
postprocessing_folder = fileparts(this_script_folder);
code_folder            = fileparts(postprocessing_folder);

% Folder that directly CONTAINS quasi_static/, dynamic_motion/ and
% contact_motion/.
data_root = fullfile(fileparts(code_folder), "data");

if ~isfolder(data_root)
    error(['Cannot find the dataset folder at:' newline ...
        char(data_root) newline ...
        'This should be the folder that directly contains ' ...
        '"quasi_static", "dynamic_motion" and "contact_motion". ' ...
        'Edit data_root above to point at it directly.']);
end

% Folder that contains data_optitrack.m, so it can be called directly.
outils_folder = fullfile(postprocessing_folder, "outils");

if ~isfolder(outils_folder)
    error(['Cannot find the outils/ folder (which should contain ' ...
        'data_optitrack.m) at:' newline char(outils_folder) newline ...
        'Edit outils_folder above to point at it directly.']);
end

addpath(outils_folder);

subset_names = {"quasi_static", "dynamic_motion", "contact_motion"};


% ====================================================================
%  ACCUMULATORS
%
%  detail_table collects one row per (recording, disk) pair.
%  invalid_frame_table collects one row per individual invalid sample,
%  if any are found. Both start empty and grow via the helper functions
%  at the bottom of this script.
% ====================================================================

detail_table = table();
invalid_frame_table = table();

total_recordings_checked           = 0;
total_disk_recording_pairs_checked = 0;
total_frame_entries_checked        = 0;
total_invalid_frame_entries_found  = 0;


% ====================================================================
%  SCAN EVERY RECORDING IN EVERY SUBSET
% ====================================================================

for subset_index = 1:numel(subset_names)

    subset_name = subset_names{subset_index};
    subset_folder = fullfile(data_root, subset_name);

    if ~isfolder(subset_folder)
        % This subset is not present at data_root -- skip it and keep going
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
            continue
        end

        % use_resense controls whether data_optitrack.m expects a sixth
        % disk (the Resense/HEX12 wand, used only in contact
        % recordings). Determined per recording from whether it actually
        % has a dataResenseFT.csv file
        resense_file = fullfile(recording_folder, "dataResenseFT.csv");
        use_resense_this_recording = isfile(resense_file);

        [N_disks, timestamps, poses_disks, rel_poses_disks, ...
            rel_kinematics_disks, is_valid_disk] = ...
            data_optitrack(optitrack_file, use_resense_this_recording);

        total_recordings_checked = total_recordings_checked + 1;
        n_time_this_recording = size(is_valid_disk, 1);

        for disk_index = 1:N_disks

            % The CSV columns are named disk_0, disk_1, ... (0-indexed)
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
                % recording. Record which samples, so they can be
                % located in the raw file.
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


% ====================================================================
%  SAVE THE DETAILED TABLE
% ====================================================================

output_folder = fullfile(data_root, "postprocess_calibration");
if ~isfolder(output_folder)
    mkdir(output_folder);
end

detail_csv_path = fullfile(output_folder, "detailed_optitrack_validity.csv");
writetable(detail_table, detail_csv_path);

fprintf("\nDetailed (recording x disk) validity table written to:\n  %s\n", detail_csv_path);


% ====================================================================
%  SAVE THE LIST OF INVALID FRAMES, IF ANY WERE FOUND
% ====================================================================

if height(invalid_frame_table) > 0
    invalid_csv_path = fullfile(output_folder, "invalid_optitrack_frames.csv");
    writetable(invalid_frame_table, invalid_csv_path);
    fprintf("List of every individual invalid frame written to:\n  %s\n", invalid_csv_path);
else
    fprintf("No invalid frames were found anywhere, so no invalid-frame list was written.\n");
end


% ====================================================================
%  FINAL SUMMARY
% ====================================================================

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
        'invalid_optitrack_frames.csv for the full list of exactly ' ...
        'which recording, disk and sample each one is.\n'], ...
        total_invalid_frame_entries_found, total_frame_entries_checked);
end


% ====================================================================
%  LOCAL FUNCTIONS
% ====================================================================

function one_row = make_detail_row(subset_name, recording_name, disk_number, ...
        n_time, n_valid, n_invalid)
    % Builds one row of the detailed table: for one disk, in one
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
    % Builds one row identifying a single invalid (occluded) sample:
    % which recording, which disk, which row of the raw CSV file
    % (sample_index), and its timestamp.
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
