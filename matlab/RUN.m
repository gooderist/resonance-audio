
%restoredefaultpath
%clear all; close all;

addpath('hrtf_data/sadie');
addpath('binaural_renderer');
addpath('ambisonics/ambix');
addpath('ambisonics/shelf_filters');
addpath('test_will/API_MO');

pkg load netcdf; % NOTE(will): needed to run on windows

% TODO(will): convert to function
sofa_filename = './test_will/hrtf_internal_interpolation_with_fadeout_1024_rev_gain_1_HATS_norm_kapil_16384.sofa';
hrir_dirname = 'test_will/kapil_16k';
in_samplerate = 44100;
in_bitrate = 16;
ambisonic_order = 5;
ambishelf_filter = true;

% Test - rip all HRIRs out of sofa file
convert_sofa_to_wav(sofa_filename, hrir_dirname, in_samplerate, in_bitrate);

% Load HRIRs (filter angles)
% issue with HANNING M must be positive on 16k sofa
%hrirdir = loadsadie(ambisonic_order, hrir_dirname);

% Convert HRIRs to SH
sadieshhrirs(ambisonic_order, ambishelf_filter, hrir_dirname);

fprintf('Finished processing');