%{
Copyright 2018 Google Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS-IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
%}

function [savedir] = loadsadie(ambisonicOrder, name)
%LOADSADIE Loads, tests and saves SADIE HRIR WAVs from /third_party.
%  Reads and tests SADIE HRIR WAVs from /third_party and saves them using
%   the following convention: EXX.XX_AYY.YY_DZZ.ZZwav, where:
%
%   XX.YY  - Elevation angle in degrees (up to 4 significant digits
%            allowed).
%   YYY.YY - Azimuth angle in degrees (up to 5 significant digits allowed).
%   ZZ.ZZ  - Measurement distance in meters (up to 3 significant digits
%            allowed).
%
%   This step is required for further Spherical Harmonic HRIR processing.
%
%   Usage: In the Matlab command prompt type loadsadie(ambisonicOrder) to
%          generate a folder with 'raw' HRIRs for the specified
%          'ambisonicOrder'.
%
%   input:
%   ambisonicOrder - Ambisonic order (max supported order is 5).
%

% Target sampling rate.
TARGET_SAMPLE_RATE = 44100;
pkg load signal;
% Tolerated error maring (resulting from changing bit resolution of the
% HRIRs).
EPSILON = 0.01;

if(~exist('name','var'))
  name = 'WILL_HRIR_RAW';
end
sadieFilesFolder = ['./' name '/'];

sadieFiles = dir([sadieFilesFolder, '*.wav']);

switch ambisonicOrder
    case 1 % Vertices of a cube (8)
        load('symmetric_cube.mat');
        savedir = [name '_symmetric_cube'];
    case 2 % Faces of a dodecahedron (12)
        load('symmetric_dodecahedron_faces.mat');
        savedir = [name '_symmetric_dodecahedron_faces'];
    case 3 % Vertices of the Lebedev26 grid (26)
        %load('3rd_lebedev26.mat');
        %savedir = [name '_lebedev26'];
        load('3rd_16channelspherepacking.mat'); % NOTE(will): testing 16 point
        savedir = [name '_16chsph'];
    case 4 % Vertices of a Pentakis Dodecahedron (32)
        load('symmetric_pentakis_dodecahedron.mat');
        savedir = [name '_symmetric_pentakis_dodecahedron'];
    case 5 % Vertices of a Pentakis Icosidodecahedron (42)
        load('5th_lebedev50.mat');
        savedir = [name '_lebedev50'];
        %load('5th_pentakis_icosi.mat');
        %savedir = [name '_pentakis_icosi'];
    otherwise
        error('Unsupported Ambisonic order');
end

if (exist(savedir, 'dir') == 0)
    mkdir(savedir);
end


for fileIndex = 1:length(sadieFiles)
    currentTestFile = [sadieFilesFolder, sadieFiles(fileIndex).name];
    testPhraseAzi = '_A';
    aziLength = 2;
    testPhraseAziIndex = strfind(currentTestFile, testPhraseAzi);
    testPhraseEle = 'E';
    eleLength = 1;
    testPhraseEleIndex = strfind(currentTestFile, testPhraseEle);
    testPhraseDFC = '_D1';
    testPhraseDFCIndex = strfind(currentTestFile, testPhraseDFC);
    el = currentTestFile(testPhraseEleIndex + ...
        eleLength:testPhraseAziIndex - 1);
    az = currentTestFile(testPhraseAziIndex + ...
        aziLength:testPhraseDFCIndex - 1);
    % NOTE(will): round off because the following expects integral values
    elRound = num2str(round(str2num(el)));
    azRound = num2str(round(str2num(az)));
    disp(['testing: Azimuth ', az, '; Elevation ', el]);
    for j = 1:length(angles)
        %if (strcmp(azRound, num2str(angles(j,1))) && ...
        %        strcmp(elRound, num2str(angles(j,2))))
        if ((abs(str2num(az) - angles(j,1)) < 1.1) && ...
            (abs(str2num(el) - angles(j,2)) < 1.1)) % NOTE(will): 7.5 seems to be good get the closest values we can find
            disp('Found it!');
            
            inputAudio = audioread(currentTestFile);
            % Apply tapering to the HRIRs.
            WINDOW_LENGTH = size(inputAudio, 1) / 10; % NOTE(will): was 8; back during 48000?
            window = hann(WINDOW_LENGTH);
            fadeOut = ...
                repmat(window(end - WINDOW_LENGTH / 2 + 1:end), 1, 2);
            inputAudio(end - WINDOW_LENGTH / 2 + 1:end, :) = ...
                inputAudio(end - WINDOW_LENGTH / 2 + 1:end, :) .* fadeOut;
            % Distance is set to 1m.
            d = '1';
            % If HRIR is in the sagittal plane, save only the left channel
            % as mono.
            % NOTE(will): use full data, not symmetrical
            %{
            if (strcmp(az, "0.0") || strcmp(az, "180.0"))
                audiowrite([savedir,'/E',el,'_A',az,'_D',d,'.wav'], ...
                    inputAudio(:,1), TARGET_SAMPLE_RATE, ...
                    'BitsPerSample', 16)
            else
                    %}
                audiowrite([savedir,'/E',el,'_A',az,'_D',d,'.wav'], ...
                    inputAudio, TARGET_SAMPLE_RATE, 'BitsPerSample', 16)
            %end
        end
    end
end
%{
% Test loaded HRIRs.
for i = 1:length(angles)
  sadieOriginalWav = audioread([sadieFilesFolder, '/azi_', ...
    num2str(angles(i, 1)), '_ele_', num2str(angles(i, 2)), '_DFC.wav']);
  sadieGoogleWav = audioread([savedir, '/E', ...
    num2str(angles(i,2)),'_A', num2str(angles(i, 1)), '_D1.wav']);
  % If the HRIRs are the same, their difference will result in a sequence
  % of 0s. Because HRIRs in the sagittal plane are mono, we have to take
  % only the left channel of the original HRIR.
  if angles(i, 1) == 0 || angles(i, 1) == 180
    difference = sadieOriginalWav(:, 1) - sadieGoogleWav;
  else
    difference = sadieOriginalWav - sadieGoogleWav;
  end
  cumSum = sum(difference(:));
  assert(cumSum < EPSILON);
end
%}
disp('Success!');

